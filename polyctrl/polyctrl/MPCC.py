import casadi as ca
import numpy as np

class MPCC:
    def __init__(self, L, K, nx, nu, ny, dt):
        self.L = L
        self.K = K
        self.nx = nx
        self.nu = nu
        self.ny = ny
        self.dt = dt

        self.solver = None
        self.u_prev = np.zeros((nu,))
        self.v_prev = 0.0

        self._gen_splines = False

        self.x_buffer = []
        self.u_buffer = []
        self.theta_buffer = []

    def set_reference_spline(self, theta, x_ref, y_ref):
        self.spline_x = ca.interpolant('Xref', 'bspline', [theta], x_ref)
        self.spline_y = ca.interpolant('Yref', 'bspline', [theta], y_ref)
        dx_ref = np.gradient(x_ref, theta)
        dy_ref = np.gradient(y_ref, theta)
        self.spline_dx = ca.interpolant('dXref', 'bspline', [theta], dx_ref)
        self.spline_dy = ca.interpolant('dYref', 'bspline', [theta], dy_ref)


        self.theta_min = float(theta[0])
        self.theta_max = float(theta[-1])
        self._gen_splines = True


    def get_reference(self, theta):
        x_ref = self.spline_x(theta)
        y_ref = self.spline_y(theta)
        dx = self.spline_dx(theta)
        dy = self.spline_dy(theta)
        phi = ca.atan2(dy, dx)
        return x_ref, y_ref, phi


    def contouring_error(self, X, Y, theta_A):
        x_ref, y_ref, phi = self.get_reference(theta_A)
        e_c = ca.sin(phi) * (X - x_ref) - ca.cos(phi) * (Y - y_ref)
        e_l = -ca.cos(phi) * (X - x_ref) - ca.sin(phi) * (Y - y_ref)
        return e_c, e_l

    def dynamics(self, x, u):
        beta = ca.atan(ca.tan(u[1]) / 2.0)
        dx = u[0] * ca.cos(x[2] + beta)
        dy = u[0] * ca.sin(x[2] + beta)
        dpsi = u[0] / self.L * ca.tan(u[1]) * ca.cos(beta)
        return ca.vertcat(dx, dy, dpsi)

    def generate_solver(self):
        assert self._gen_splines, "Reference spline must be set before generating the solver!"

        X = ca.SX.sym('X', self.nx, self.K + 1)  
        U = ca.SX.sym('U', self.nu, self.K)      
        V = ca.SX.sym('V', self.K)              
        ThetaA = ca.SX.sym('ThetaA', self.K + 1) 

        q_c = ca.SX.sym('q_c')
        q_l = ca.SX.sym('q_l')
        gamma = ca.SX.sym('gamma')
        R_u = ca.SX.sym('R_u', self.nu)
        R_v = ca.SX.sym('R_v')
        R_u_prev = ca.SX.sym('R_u_prev', self.nu)
        R_v_prev = ca.SX.sym('R_v_prev')
        u_ref = ca.SX.sym('u_ref', self.nu)
        R_ref = ca.SX.sym('R_ref', self.nu)


        x0 = ca.SX.sym('x0', self.nx)           
        theta0 = ca.SX.sym('theta0')            
        u_prev = ca.SX.sym('u_prev', self.nu)   
        v_prev = ca.SX.sym('v_prev')            

        obstacle = ca.SX.sym('obstacle', 3)
        obj = 0

        for k in range(self.K):
            e_c, e_l = self.contouring_error(X[0, k], X[1, k], ThetaA[k])

            obj += q_c * (e_c)**2 + q_l * (e_l)**2
            obj += -gamma * V[k] * self.dt
            if k == 0:
                obj += (U[:,k] - u_prev).T @ ca.diag(R_u_prev) @ (U[:,k] - u_prev)
                obj += (V[k] - v_prev)**2 * R_v_prev[0]
                obj += (U[:,k] - u_ref).T @ ca.diag(R_ref) @ (U[:,k] - u_ref)

            else:
                obj += (U[:,k] - U[:,k-1]).T @ ca.diag(R_u) @ (U[:,k] - U[:,k-1])
                obj += (V[k] - V[k-1])**2 * R_v[0]

            


        g = []
        g.append(X[:,0] - x0)           
        g.append(ThetaA[0] - theta0)     

        for k in range(self.K):
            x_next = X[:,k] + self.dt * self.dynamics(X[:,k], U[:,k])
            g.append(X[:,k+1] - x_next)

            theta_next = ThetaA[k] + V[k] * self.dt
            g.append(ThetaA[k+1] - theta_next)

        L =  0.256
        x_obs = obstacle[0]
        y_obs = obstacle[1]
        scale_obs = obstacle[2]
        for i in range(1, self.K+1):
            g.append(((X[0, i] - x_obs)**2 + (X[1, i]- y_obs)**2) - (scale_obs/3 +L/3 )**2)

        g = ca.vertcat(*g)

        opt_variables = ca.vertcat(
            ca.vec(X), 
            ca.vec(U), 
            V, 
            ThetaA
        )

        p = ca.vertcat(x0, theta0, u_prev, v_prev, q_c, q_l, gamma, R_u, R_v, R_u_prev, R_v_prev, u_ref, R_ref, obstacle)

        nlp = {'f': obj, 'x': opt_variables, 'p': p, 'g': g}

        opts = {'ipopt.print_level': 0, 'ipopt.max_iter': 1000, 'print_time': 0}
        self.solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

        self.opt_variables = opt_variables
        self.X = X
        self.U = U
        self.V = V
        self.ThetaA = ThetaA
        self.g = g

    def solve(self, x0, theta0, u_prev, v_prev,
             q_c=1, q_l=10, gamma=1, R_u=[1, 1], R_v=[1], R_u_prev=[1, 1], R_v_prev=[1],
             v_max = 2.0,  
             x_min = [-20, -20, -np.inf],
             x_max = [20, 20, np.inf],
             u_min = [0.2, -np.pi/6],
             u_max = [2.0, np.pi/6],
             u_ref = [0.2, 0.0],
             R_ref = [0.1, 0.0],
             obstacle = [1000.,1000.,1.]
            ):
        n_vars = self.opt_variables.shape[0]
        x_init = np.zeros((n_vars,))



        lbx = []
        ubx = []
        for _ in range(self.K + 1):
            lbx += x_min
        for _ in range(self.K):
            lbx += u_min
        lbx += [0.0] * self.K           
        lbx += [0.0] * (self.K + 1)      

        for _ in range(self.K + 1):
            ubx += x_max
        for _ in range(self.K):
            ubx += u_max
        ubx += [v_max] * self.K          
        ubx += [self.theta_max] * (self.K + 1)


        lbg = [0] * self.g.shape[0] 
        ubg = [0] * (self.g.shape[0]-self.K) + [np.inf]*self.K


        sol = self.solver(
            x0=x_init,
            p=np.concatenate((x0, theta0, u_prev, v_prev, q_c, q_l, gamma, R_u, R_v, R_u_prev, R_v_prev, u_ref, R_ref, obstacle)),
            lbg=lbg,
            ubg=ubg,
            lbx=lbx,
            ubx=ubx
        )

        opt_sol = sol['x']

        offset_X = self.nx * (self.K + 1)
        offset_U = offset_X + self.nu * self.K
        offset_V = offset_U + self.K

        X_opt = np.array(ca.reshape(opt_sol[:offset_X], self.nx, self.K+1))
        U_opt = np.array(ca.reshape(opt_sol[offset_X:offset_U], self.nu, self.K))
        V_opt = np.array(opt_sol[offset_U:offset_V])
        theta_opt = np.array(opt_sol[offset_V:])

        self.x_buffer.append(X_opt)
        self.u_buffer.append(U_opt)
        self.theta_buffer.append(theta_opt)

        return U_opt[:,0], V_opt[0]
