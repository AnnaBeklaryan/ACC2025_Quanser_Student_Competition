#!/usr/bin/env python3
import numpy as np
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from sensor_msgs.msg import JointState
from qcar2_interfaces.msg import MotorCommands

from tf2_ros import Buffer, TransformListener, TransformException
import tf_transformations

from utils.mats import SDCSRoadMap
from .MPCC import MPCC

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import String

import signal
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters
from scipy.interpolate import splprep, splev, CubicSpline

import argparse
import sys

from qcar2_interfaces.msg import BooleanLeds

class mpc_node(Node):
    
    def __init__(self,  with_obstacle=False):
        super().__init__('mpc_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        qos = QoSProfile(
            depth=1,
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.detection_labels = []
        self.stop_waiting = False
        self.stop_cooldown = False
        self.stop_index = 0
        self.last_stop_time = None
        self.last_stop_cooldown_time = None
        self.cross_waiting= False
        self.last_cross_time = None
        self.cross_cooldown =False

        self.map_boundary = [-1, 1, -1, 1] 

        self.qanak = 0
        self.flag = False
        self.jaman = None
        self.with_obstacle = with_obstacle
        self.curr_time = None
        self.motor_command_publisher = self.create_publisher(MotorCommands, 'qcar2_motor_speed_cmd', 1)
        self.velocity_subscriber = self.create_subscription(JointState, '/qcar2_joint', self.process_velocity, 10)
        self.detection_subscriber = self.create_subscription(String, '/yolo_det', self.detection_callback, 10)

        self.led_publisher_ = self.create_publisher(BooleanLeds, 'qcar2_led_cmd', 10)

        self.round_sign_detected_time = None
        self.round_sign_led_active = False

        self.round_sign_led_blinking = False
        self.round_sign_blink_start_time = None
        self.round_sign_last_toggle_time = None
        self.round_sign_led_on = False  
        self.round_blink_interval = 0.2 

        self.right_sign_led_blinking = False
        self.right_sign_blink_start_time = None
        self.right_sign_last_toggle_time = None
        self.right_sign_led_on = False
        self.right_blink_interval = 0.2  

        self.marker_pub = self.create_publisher(Marker, "waypoint_marker", qos)
        self.traj_marker_pub = self.create_publisher(Marker, "mpc_trajectory", qos)
        self.pred_marker_pub = self.create_publisher(Marker, "mpc_prediction", qos)
        self.ref_marker_pub = self.create_publisher(Marker, "mpc_reference", qos)

        self.led_param_client = self.create_client(
            SetParameters,
            '/qcar2_hardware/set_parameters'
        )
        while not self.led_param_client.wait_for_service(timeout_sec=1.):
            self.get_logger().info('Waiting for qcar2_hardware parameter service...')

        self.x0 = None
        while self.x0 is None:
            self.update_pose_from_tf()
            rclpy.spin_once(self)

        self.start_delay = 0.1
        self.L = 0.256
        self.controllerUpdateRate = 10
        self.dt = 1 / self.controllerUpdateRate

        self.nx, self.nu, self.ny = 3, 2, 2
        self.K = 10
        
        self.u_min = [0, -np.pi / 6]
        self.u_max = [2, np.pi / 6]
        self.deltau_min = [-1, -2]
        self.deltau_max = [1, 2]

        self.x_min = [-20, -20, -np.inf]
        self.x_max = [20, 20, np.inf]


        self.flag_p = False
        self.flag_d = False
        self.stop_init_p = 0.
        self.led_values = None
        self.flag_c = False

        self.roadmap = SDCSRoadMap(leftHandTraffic=False)
        
        self.first_pose = self.roadmap.add_node([-1.205,-0.83, -44.7 % (2*np.pi)])
        first_node = len(self.roadmap.nodes) - 1
        self.roadmap.add_edge(first_node,2,radius=0.0)
        self.roadmap.add_edge(10, first_node,radius=1.48202)
        self.roadmap.add_edge(first_node, 1, radius=0.866326)


        self.nodeSequence = [24,20,9,10]

        waypointSequence = self.roadmap.generate_path(self.nodeSequence, spacing=0.001, scale_factor=[1.01, 1.0])


        theta = 0.7177 
        R = np.array([[np.cos(theta), - np.sin(theta) ],[np.sin(theta) , np.cos(theta)]])

        x_init = [-1.205, -0.83]

        self.waypointSequence_tf = np.array([R @ (waypointSequence[: , i] - x_init)  for i in range(waypointSequence.shape[1])]).T 

 
        self.goal_tresh = 0.4
        self.pick_up =  np.array([0.125, 4.395]) 
        self.drop_off = np.array([-1.198, 0.795]) 
        self.obstacle = np.array([-1.977, 2.973, 0.24]) 
        self.pick_up = R @ (self.pick_up - x_init)
        self.drop_off = R @ (self.drop_off - x_init)
        self.obstacle[:2] = R @ (self.obstacle[:2] - x_init)

        self.pedastrian = np.array([-3.09, 2.46])
        self.pick_up_index = 2
        self.flag_p = False
        self.flag_d = False
        self.flag_c = False
        self.time_cone = 0

        signal.signal(signal.SIGINT, self.shutdown_callback)
        self.is_shutting_down = False

        waypoints = self.waypointSequence_tf
        dists = np.sqrt(np.sum(np.diff(waypoints, axis=1)**2, axis=0))
        self.arc_lengths = np.insert(np.cumsum(dists), 0, 0)
        self.spline_x = CubicSpline(self.arc_lengths, waypoints[0, :])
        self.spline_y = CubicSpline(self.arc_lengths, waypoints[1, :])

        x = self.spline_x(self.arc_lengths)
        y = self.spline_y(self.arc_lengths)
        dx = np.gradient(x, self.arc_lengths)
        dy = np.gradient(y, self.arc_lengths)       
        ddx = np.gradient(dx, self.arc_lengths)
        ddy = np.gradient(dy, self.arc_lengths)

        numerator = dx * ddy - dy * ddx
        denominator = (dx**2 + dy**2)**1.5 + 1e-8
        self.curvature = np.abs(numerator / denominator)

        self.mpcc = MPCC(
            L=self.L, K=self.K, nx=self.nx, nu=self.nu, ny=self.ny, dt=self.dt
        )

        self.mpcc.set_reference_spline(self.arc_lengths, waypoints[0, :], waypoints[1, :])
        self.mpcc.generate_solver()

        self.u_mpc = np.array([0.0, 0.0])
        self.v_mpc = np.array([0.0])

        self.t0 = time.time()
        self.t = 0
        self.v, self.delta, self.throttle, self.v_theta = 0., 0., 0., 0.
        self.position = [0.0, 0.0]
        self.yaw = 0.
        self.theta_A = np.array([0.0])
        self.theta_idx = 0
        
        self.min_distance = 0.
        self.object_angle = 0.
        self.publish_waypoints_marker()
        
        self.timer = self.create_timer(self.dt, self.loop)

    def publish_pickup_dropoff_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"  
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pick_up_drop_off"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.1 
        marker.scale.y = 0.1
        marker.color.a = 1.0  

        marker.color.r = 1.0
        pick_up_point = Point()
        pick_up_point.x = self.pick_up[0]
        pick_up_point.y = self.pick_up[1]
        pick_up_point.z = 0.0  
        marker.points.append(pick_up_point)

        marker.color.b = 1.0 
        drop_off_point = Point()
        drop_off_point.x = self.drop_off[0]
        drop_off_point.y = self.drop_off[1]
        drop_off_point.z = 0.0
        marker.points.append(drop_off_point)


        marker.color.b = 1.0 
        obstacle_point = Point()
        obstacle_point.x = self.obstacle[0]
        obstacle_point.y = self.obstacle[1]
        obstacle_point.z = 0.0
        marker.points.append(obstacle_point)

        self.marker_pub.publish(marker)

    def reached_drop_off(self):
        self.dist_drop_off = np.linalg.norm([self.position[0] - self.drop_off[0], self.position[1] - self.drop_off[1]])

        if self.dist_drop_off <= 0.2:
            if not self.flag:
                self.jaman = time.time()
                self.qanak += 1
                self.flag = True

            if self.curr_time - self.jaman <= 3:
                

                return True
        return False

        

    def resample_waypoints(self, path, spacing=0.1):
        tck, u = splprep([path[0], path[1]], s=0)
        unew = np.linspace(0, 1, int(np.linalg.norm(np.diff(path, axis=1), axis=0).sum() / spacing))
        out = splev(unew, tck)
        return np.vstack(out)
        
    def shutdown_callback(self, signum, frame):
        self.is_shutting_down = True         
        self.get_logger().info("Node interrupted")
        
        self.publish_commands(0., 0.)
        print("Stopping robot")
        self.timer.cancel()
        rclpy.shutdown()

    def set_led_color(self, color_id: int):
        if hasattr(self, 'last_led_color') and self.last_led_color == color_id:
            return  
        self.last_led_color = color_id

        param_msg = Parameter()
        param_msg.name = 'led_color_id'
        param_msg.value = ParameterValue(
            type=ParameterType.PARAMETER_INTEGER,
            integer_value=color_id
        )

        request = SetParameters.Request()
        request.parameters = [param_msg]
        self.led_param_client.call_async(request)


    def process_velocity(self, msg: JointState):
        self.v = (msg.velocity[0] / (720.0 * 4.0)) * ((13.0 * 19.0) / (70.0 * 30.0)) * (2.0 * np.pi) * 0.033

    def publish_waypoints_marker(self):

        marker = Marker()
        marker.header.frame_id = "map"  
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()


        for i in range(self.waypointSequence_tf.shape[1]):
            p = Point()
            p.x = float(self.waypointSequence_tf[0, i])
            p.y = float(self.waypointSequence_tf[1, i])
            p.z = 0.0
            marker.points.append(p)

        self.marker_pub.publish(marker)



    def publish_marker(self, points, publisher, frame_id="map", color=(1, 0, 0), ns="default"):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = color
        points = np.array(points)

        if points.ndim == 1:
            points = points.reshape(2, 1)

        marker.points = [
            Point(x=float(x), y=float(y), z=0.0)
            for x, y in points.T
            if np.isfinite(x) and np.isfinite(y)
        ]
        publisher.publish(marker)

    def detection_callback(self, msg):
        self.detection_labels = []
        for det in msg.data.split(', '):
            try:
                label, conf = det.split(':')
                self.detection_labels.append((label.upper(), float(conf)))
            except ValueError:
                continue
        

    def turn_on_left_lights_and_signals(self):
        led_commands = BooleanLeds()
        led_commands.led_names = [
            "left_outside_brake_light", "left_inside_brake_light", "left_reverse_light",
            "left_rear_signal", "left_outside_headlight", "left_middle_headlight", 
            "left_inside_headlight", "left_front_signal"
        ]
        led_commands.values = [True] * 8 
        self.led_publisher_.publish(led_commands)


    def turn_off_left_lights_and_signals(self):
        led_commands = BooleanLeds()
        led_commands.led_names = [
            "left_outside_brake_light", "left_inside_brake_light", "left_reverse_light",
            "left_rear_signal", "left_outside_headlight", "left_middle_headlight", 
            "left_inside_headlight", "left_front_signal"
        ]
        led_commands.values = [False] * 8 
        self.led_publisher_.publish(led_commands)



    def turn_on_right_lights_and_signals(self):
        led_commands = BooleanLeds()
        led_commands.led_names = [
            "right_outside_brake_light", "right_inside_brake_light", "right_reverse_light",
            "right_rear_signal", "right_outside_headlight", "right_middle_headlight", 
            "right_inside_headlight", "right_front_signal"
        ]
        led_commands.values = [True] * 8 
        self.led_publisher_.publish(led_commands)


    def turn_off_right_lights_and_signals(self):
        led_commands = BooleanLeds()
        led_commands.led_names = [
            "right_outside_brake_light", "right_inside_brake_light", "right_reverse_light",
            "right_rear_signal", "right_outside_headlight", "right_middle_headlight", 
            "right_inside_headlight", "right_front_signal"
        ]
        led_commands.values = [False] * 8  
        self.led_publisher_.publish(led_commands)



    def get_filtered_detections(self, target_label, min_conf=0.88, max_conf=0.89):
        return [
            (label, conf)
            for label, conf in self.detection_labels
            if label == target_label and conf >= min_conf 
        ]

    def reached_cross_section(self):
        if self.jaman is not None:
            if self.curr_time - self.jaman >= 3 and self.dist_drop_off <= 0.4:
                red_detects = self.get_filtered_detections('RED', 0.83, 0.9)
            else:
                red_detects = []

        else:
            red_detects = []        

        green_detects = self.get_filtered_detections('GREEN', 0.84, 0.9)

        if self.cross_waiting:
            if len(green_detects) > 0:
                self.get_logger().info("GREEN light detected — proceeding.")
                self.cross_waiting = False
                self.cross_cooldown = True
                self.last_cross_cooldown_time = time.time()
                return False

            else:
                return True  

        if self.cross_cooldown:
            if len(red_detects) == 0 or (time.time() - self.last_cross_cooldown_time > 10.0):
                self.get_logger().info("Cross-section cooldown complete.")
                self.cross_cooldown = False
            return False

        if len(red_detects) > 0 and not self.cross_waiting:
            self.get_logger().info("RED light detected — waiting for GREEN.")
            self.cross_waiting = True
            self.last_cross_time = time.time()
            return True

        return False
    def is_round_detected(self):
        return len(self.get_filtered_detections('ROUND', 0.9,0.92)) > 0

    def cone_detected(self):

        return len(self.get_filtered_detections('CONE', 0.8,0.99)) > 0

    def is_person_detected(self):
        return len(self.get_filtered_detections('PERSON', 0.7,0.81)) > 0

    def reached_end(self):
        dist_from_end = abs(self.arc_lengths[-1] - self.theta_A)
        if dist_from_end <= 0.7:
            return True
        else:
            return False


    def reached_stop_sign(self):
        stop_detects = self.get_filtered_detections('STOP', 0.91, 0.91)
        now = time.time()

        if self.stop_waiting:
            if now - self.last_stop_time < 3.0:
                return True
            else:
                self.get_logger().info(f"Finished STOP #{self.stop_index}.")
                self.stop_waiting = False
                self.stop_cooldown = True
                self.last_stop_detect_time = now
                return False

        if self.stop_cooldown:
            if (now - self.last_stop_detect_time > 15.0):
                self.get_logger().info("Cooldown over. Looking for new STOP.")
                self.stop_cooldown = False
            return False

        if len(stop_detects) > 0 and not self.stop_waiting:
            self.get_logger().info(f"STOP sign #{self.stop_index + 1} detected. Stopping...")
            self.stop_waiting = True
            self.last_stop_time = now
            self.stop_index += 1
            return True

        return False



    def update_pose_from_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())

            x_map = transform.transform.translation.x
            y_map = transform.transform.translation.y

            q = transform.transform.rotation
            euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            yaw_map = euler[2]
    
            self.x0 = np.array([x_map, -y_map, yaw_map])
            x_world = x_map
            y_world = y_map

            self.position = [x_world, y_world]
            self.yaw = yaw_map
        except TransformException as ex:
            pass

    def publish_commands(self, steering_angle, throttle):
        msg = MotorCommands()
        msg.motor_names = ["steering_angle", "motor_throttle"]
        msg.values = [float(steering_angle), float(throttle)]
        self.motor_command_publisher.publish(msg)


    def project_to_path(self, x, y, theta_array, ref):
        idx_len = 1000
        if self.theta_idx + idx_len <= ref.shape[1]:
            distances = np.sqrt((x-ref[0,self.theta_idx:self.theta_idx+idx_len])**2 + (y-ref[1,self.theta_idx:self.theta_idx+idx_len])**2)
        else:
            distances = np.sqrt((x-ref[0,self.theta_idx:])**2 + (y-ref[1,self.theta_idx:])**2)

        min_idx = np.argmin(distances)
        self.theta_idx += min_idx
        return np.array([theta_array[self.theta_idx]])

    def loop(self):
        tp = self.t
        self.t = time.time() - self.t0
        self.dt = self.t-tp
        self.update_pose_from_tf()
        self.curr_time = time.time()
        self.publish_pickup_dropoff_marker()

        x, y, th = self.position[0], self.position[1], self.yaw
        self.theta_A = self.project_to_path(x, y, self.arc_lengths, self.waypointSequence_tf)
        v = self.v

        if self.t < self.start_delay:
            self.set_led_color(1)
            self.throttle = 0
            self.u_mpc = [0, 0]
            self.delta = 0
            
        else:
            
            self.u_prev = self.u_mpc
            self.v_prev = self.v_mpc
            self.u_prev[0] = v

    
            dist_pick_up = np.linalg.norm([x - self.pick_up[0], y - self.pick_up[1]])
            dist_cone = np.linalg.norm([x - self.obstacle[0], y - self.obstacle[1]])
            
            if dist_cone <= 0.8:
                if not self.flag_c and self.cone_detected():
                    self.time_cone = time.time()
                self.flag_c = True

            if dist_pick_up <= self.goal_tresh:
                if not self.flag_p and self.is_person_detected() and self.pick_up_index %2 == 0:
                    self.stop_init_p = time.time()
                    self.pick_up_index +=1
                self.flag_p = True

            if ((self.flag_p)  and (self.curr_time - self.stop_init_p)<=3) :
                self.set_led_color(2)
                self.throttle = 0
                self.delta = 0   
            elif self.reached_drop_off():
                self.set_led_color(4)
                self.throttle = 0
                self.delta = 0

            elif self.reached_cross_section():
                self.set_led_color(3)
                self.throttle = 0
                self.delta = 0
                
            elif self.reached_stop_sign():
                self.set_led_color(0)
                self.throttle = 0
                self.delta = 0     

            elif self.reached_end():
                self.set_led_color(1)
                self.throttle = 0
                self.delta = 0                  
            else:
                self.flag_p = False
                self.flag_d = False
        
                self.reached_t = False
                self.set_led_color(1)
             

                q_c=[1.8]
                q_l=[7]
                gamma=[1]
                R_u=[0.005, 1.1]
                R_v=[0.5]
                R_u_prev=[0.005, 1.1]
                R_v_prev=[0.5]
                v_max = 2.0
                u_min = [0.0, -np.pi/6]
                u_max = [2.0, np.pi/6]

                R_ref = [17, 0.05]

                u_ref_max = 0.7
                u_ref_val = np.clip(u_ref_max * np.exp(-0.4* self.curvature[self.theta_idx]), 0.0, 2.0)
                
                u_ref = [u_ref_val, 0]
                current_obstacle = [1000, 1000, 0.01]

                if self.t<=3:
                    q_c=[1]
                    q_l=[10]
                    gamma=[5]
                    R_u=[0.01, 0.05]
                    R_v=[0.001]
                    R_u_prev=[0.01, 0.05]
                    R_v_prev=[0.001]
                    v_max = 2.0
                    u_min = [0.0, -np.pi/6]
                    u_max = [2.0, np.pi/6]

                    R_ref = [5, 0]

                    u_ref_max = 0.2
                    u_ref_val = np.clip(u_ref_max * np.exp(-5 * self.curvature[self.theta_idx]), 0.2, 2.0)
                    
                    u_ref = [u_ref_val, 0]

                elif self.with_obstacle and self.flag_c:
                    current_obstacle = self.obstacle.copy()
                    if self.curr_time - self.time_cone<=5:
                        q_c=[1]
                        q_l=[7]
                        gamma=[1]
                        R_u=[0.005, 0.4]
                        R_v=[0.5]
                        R_u_prev=[0.005, 0.4]
                        R_v_prev=[0.5]
                        v_max = 2.0#
                        u_min = [0.0, -np.pi/6]#
                        u_max = [2.0, np.pi/6]#

                        R_ref = [17, 0.05]

                        u_ref_max = 0.4
                        u_ref_val = np.clip(u_ref_max * np.exp(-0.4* self.curvature[self.theta_idx]), 0.4, 2.0)
                        
                        u_ref = [u_ref_val, 0]
                    else:
                        self.flag_c = False 

                self.u_mpc, self.v_mpc = self.mpcc.solve(
                np.array([x, y, th]), self.theta_A, self.u_prev, self.v_prev,
                            q_c=q_c, q_l=q_l, gamma=gamma, R_u=R_u, R_v=R_v, R_u_prev=R_u_prev, R_v_prev=R_v_prev,
                            v_max=v_max, u_min=u_min, u_max=u_max, u_ref=u_ref, R_ref=R_ref, obstacle=current_obstacle)
                        

                self.delta = self.u_mpc[1]
                self.throttle = self.u_mpc[0] 
                  

                self.publish_marker(self.position, self.traj_marker_pub, frame_id="map", color=(0.0, 0.0, 1.0), ns="trajectory")
                self.publish_marker(self.mpcc.x_buffer[-1][:2], self.pred_marker_pub, frame_id="map", color=(0.0, 0.0, 0.0), ns="prediction")

            if self.is_round_detected() and not self.round_sign_led_blinking and not self.right_sign_led_blinking:
                self.round_sign_blink_start_time = time.time()
                self.round_sign_last_toggle_time = time.time()
                self.round_sign_led_blinking = True
                self.round_sign_led_on = False

            if self.round_sign_led_blinking:
                elapsed = time.time() - self.round_sign_blink_start_time
                if elapsed > 5.0:
                    self.turn_off_left_lights_and_signals()
                    self.round_sign_led_blinking = False

                    self.right_sign_blink_start_time = time.time()
                    self.right_sign_last_toggle_time = time.time()
                    self.right_sign_led_blinking = True
                    self.right_sign_led_on = False
                else:
                    toggle_elapsed = time.time() - self.round_sign_last_toggle_time
                    if toggle_elapsed >= self.round_blink_interval:
                        if self.round_sign_led_on:
                            self.turn_off_left_lights_and_signals()
                        else:
                            self.turn_on_left_lights_and_signals()
                        self.round_sign_led_on = not self.round_sign_led_on
                        self.round_sign_last_toggle_time = time.time()


  

            if self.right_sign_led_blinking:
                elapsed = time.time() - self.right_sign_blink_start_time
                if elapsed > 4.0:
                    self.turn_off_right_lights_and_signals()
                    self.right_sign_led_blinking = False
                else:
                    toggle_elapsed = time.time() - self.right_sign_last_toggle_time
                    if toggle_elapsed >= self.right_blink_interval:
                        if self.right_sign_led_on:
                            self.turn_off_right_lights_and_signals()
                        else:
                            self.turn_on_right_lights_and_signals()
                        self.right_sign_led_on = not self.right_sign_led_on
                        self.right_sign_last_toggle_time = time.time()




        self.publish_commands(self.delta, self.throttle)

    def __del__(self):
        pass


def main(args=None):  
    parser = argparse.ArgumentParser(description='QCar Scenario Setup')
    parser.add_argument('--with_obstacle', action='store_true', help='With obstacle avoidance')
    args = parser.parse_args()
    rclpy.init(args=sys.argv)
    
    node = mpc_node(with_obstacle = args.with_obstacle)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.is_shutting_down:
            node.shutdown_callback(None, None)
        node.destroy_node() 

    rclpy.shutdown()

if __name__ == '__main__':
    
    main()
