#!/usr/bin/env python3
import numpy as np
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import JointState
from qcar2_interfaces.msg import MotorCommands
from tf2_ros import Buffer, TransformListener, TransformException
import tf_transformations
from hal.products.mats import SDCSRoadMap

import math
from std_msgs.msg import String
import cv2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters

import numpy as np
from qcar2_interfaces.msg import BooleanLeds
from pal.utilities.math import wrap_to_pi

class SpeedController:
    # ==============  SECTION A -  Speed Control  ====================
    def __init__(self, kp=0, ki=0, kd=0):
        self.maxThrottle = 5

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ei = 0
        self.e_prev = 0  # Store previous error for derivative calculation
     

    def update(self, v, v_ref, dt):
        # Calculate the error
        e = v_ref - v

        # Integral term - sum of errors
        self.ei += e * dt

        # Derivative term - difference of current and previous error
        de = (e - self.e_prev) / dt if dt > 0 else 0  # Avoid division by zero
        self.e_prev = e  # Update previous error for the next iteration

        # PID control output
        output = self.kp * e + self.ki * self.ei + self.kd * de

        # Clamp the output to the maximum throttle limit
        return np.clip(output, -self.maxThrottle, self.maxThrottle)
        
        
class SteeringController:

    def __init__(self, waypoints, k=1, cyclic=True):
        self.maxSteeringAngle = np.pi/6

        self.wp = waypoints
        self.N = len(waypoints[0, :])
        self.wpi = 0

        self.k = k
        self.cyclic = cyclic

        self.p_ref = (0, 0)
        self.th_ref = 0

    # ==============  SECTION B -  Steering Control  ====================
    def update(self, p, th, speed):
        wp_1 = self.wp[:, np.mod(self.wpi, self.N-1)]
        wp_2 = self.wp[:, np.mod(self.wpi+15, self.N-1)]

        v = wp_2 - wp_1
        v_mag = np.linalg.norm(v)
        try:
            v_uv = v / v_mag
        except ZeroDivisionError:
            return 0

        tangent = np.arctan2(v_uv[1], v_uv[0])

        s = np.dot(p-wp_1, v_uv)

        if s >= v_mag:
            if self.cyclic or self.wpi < self.N-2:
                self.wpi += 1
        # print(self.wpi)
        # print(p)
        ep = wp_1 + v_uv*s
        ct = ep - p
        dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)

        ect = np.linalg.norm(ct) * np.sign(dir)
        psi = wrap_to_pi(tangent-th)

        self.p_ref = ep
        self.th_ref = tangent

        return np.clip(
            wrap_to_pi(psi + np.arctan2(self.k*ect, speed)),
            -self.maxSteeringAngle,
            self.maxSteeringAngle)
        

class VehicleControlNode(Node):
    def __init__(self):
        # ==============  SECTION C -  Control Node ====================
        super().__init__('vehicle_control_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        qos = QoSProfile(
            depth=1,
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Flags
        self.detection_labels = []
        self.stop_waiting = False
        self.stop_cooldown = False
        self.stop_index = 0
        self.last_stop_time = None
        self.last_stop_cooldown_time = None
        self.last_person_detect_time = None
        self.cross_waiting= False
        self.last_cross_time = None
        self.cross_cooldown =False
        self.person_waiting = False
        self.person_cooldown = False
        self.trafic_1 = np.array([-1.08, 1.32, 0])
        self.trafic_2 = np.array([0.8, 1.3, 90])
        self.reached_t = False


        # Publishers & subscribers
        self.motor_command_publisher = self.create_publisher(MotorCommands, 'qcar2_motor_speed_cmd', 1)
        self.velocity_subscriber = self.create_subscription(JointState, '/qcar2_joint', self.process_velocity, 10)
        self.detection_subscriber = self.create_subscription(String, '/yolo_det', self.detection_callback, 10)
        self.marker_pub = self.create_publisher(Marker, "waypoint_marker", qos)
        self.led_param_client = self.create_client(
            SetParameters,
            '/qcar2_hardware/set_parameters'
        )
        while not self.led_param_client.wait_for_service(timeout_sec=1.):
            self.get_logger().info('Waiting for qcar2_hardware parameter service...')
        self.led_publisher_ = self.create_publisher(BooleanLeds, 'qcar2_led_cmd', 10)

        
        self.x0 = None
        while self.x0 is None:
            self.update_pose_from_tf()
            rclpy.spin_once(self)

        # Controller parameters
        self.velocity_kp = 0.3 #0.25
        self.velocity_ki = 0.0 #0.0
        self.velocity_kd = 0.1
        self.start_delay = 0.01
        self.v_ref = 3 #1
        self.K_stanly = 0.4 #0.8
        self.controllerUpdateRate = 100
        self.dt = 1 / self.controllerUpdateRate
        # Generate road map 
        self.roadmap = SDCSRoadMap(leftHandTraffic=False)

        self.first_pose = self.roadmap.add_node([-0.405,-1.101,0.006])
        first_node = len(self.roadmap.nodes) - 1
        self.roadmap.add_edge(first_node,2,radius=0.0)
        self.roadmap.add_edge(10, first_node,radius=1.48202)
        self.roadmap.add_edge(first_node, 1, radius=0.866326)


        self.nodeSequence = [24, 2, 4 , 14, 20, 22, 9, 7, 14, 20, 22, 10]#[24, 1, 13, 19, 17, 20, 22, 9, 7, 5, 3, 1, 8, 10, 2, 4, 6, 8, 10]

        waypointSequence = self.roadmap.generate_path(self.nodeSequence)
        theta = 0.7177 #np.deg2rad(-7.5)
        R =np.array([[np.cos(theta), - np.sin(theta) ],[np.sin(theta) , np.cos(theta)]])

        x_init = np.array([-1.205  ,-0.83 ]) 

        self.waypointSequence_tf = np.array([R @ (waypointSequence[: , i] - x_init)  for i in range(waypointSequence.shape[1])]).T 
        self.waypointSequence_tf[1, :] = self.waypointSequence_tf[1, :]

        # Controllers
        self.speedController = SpeedController(kp=self.velocity_kp, ki=self.velocity_ki, kd=self.velocity_kd)
        self.steeringController = SteeringController(waypoints=self.waypointSequence_tf, k=self.K_stanly)

        # State variables
        self.t0 = time.time()
        self.t = 0
        self.v, self.delta, self.throttle = 0., 0., 0.
        self.position = [0.0, 0.0]
        self.yaw = 0.

        self.goal_tresh = 0.07 #0.1
        self.pick_up = np.array([-2.16,5.1])
        self.drop_off = np.array([-1.08, 1.32])
        self.pedastrian = np.array([-3.17, 2.64])
        self.pick_up_index = 2

        self.flag_p = False
        self.flag_d = False
        self.flag_pedastrian = False
        self.stop_init_p = 0.
        self.stop_init_d = 0.
        self.stop_init_pedastrian = 0.  


        self.stop_init_t =0.
        self.round_sign_detected_time = None
        self.round_sign_led_active = False

        self.round_sign_led_blinking = False
        self.round_sign_blink_start_time = None
        self.round_sign_last_toggle_time = None
        self.round_sign_led_on = False  # Tracks whether lights are currently ON or OFF
        self.round_blink_interval = 0.2  # seconds (adjust for faster/slower blinking)


        # Right lights blinking state
        self.right_sign_led_blinking = False
        self.right_sign_blink_start_time = None
        self.right_sign_last_toggle_time = None
        self.right_sign_led_on = False
        self.right_blink_interval = 0.2  # seconds
        self.stopped = False
        self.stop_time = 0.
        self.publish_waypoints_marker()

        self.timer = self.create_timer(self.dt, self.control_loop)
        time.sleep(0.01)

    def shutdown_callback(self, signum, frame):
        self.is_shutting_down = True         
        self.get_logger().info("Node interrupted")
        
        self.publish_commands(0., 0.)
        print("Stopping robot")
        self.timer.cancel()
        rclpy.shutdown()

    def set_led_color(self, color_id: int):
        if hasattr(self, 'last_led_color') and self.last_led_color == color_id:
            return  # no need to resend
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

    def turn_on_left_lights_and_signals(self):
        """Activate left part lights and signals."""
        led_commands = BooleanLeds()
        led_commands.led_names = [
            "left_outside_brake_light", "left_inside_brake_light", "left_reverse_light",
            "left_rear_signal", "left_outside_headlight", "left_middle_headlight", 
            "left_inside_headlight", "left_front_signal"
        ]
        led_commands.values = [True] * 8  # Turn on all left part lights
        self.led_publisher_.publish(led_commands)


    def turn_off_left_lights_and_signals(self):
        """Deactivate left part lights and signals."""
        led_commands = BooleanLeds()
        led_commands.led_names = [
            "left_outside_brake_light", "left_inside_brake_light", "left_reverse_light",
            "left_rear_signal", "left_outside_headlight", "left_middle_headlight", 
            "left_inside_headlight", "left_front_signal"
        ]
        led_commands.values = [False] * 8  # Turn off all left part lights
        self.led_publisher_.publish(led_commands)



    def turn_on_right_lights_and_signals(self):
        """Activate left part lights and signals."""
        led_commands = BooleanLeds()
        led_commands.led_names = [
            "right_outside_brake_light", "right_inside_brake_light", "right_reverse_light",
            "right_rear_signal", "right_outside_headlight", "right_middle_headlight", 
            "right_inside_headlight", "right_front_signal"
        ]
        led_commands.values = [True] * 8  # Turn on all left part lights
        self.led_publisher_.publish(led_commands)


    def turn_off_right_lights_and_signals(self):
        """Deactivate left part lights and signals."""
        led_commands = BooleanLeds()
        led_commands.led_names = [
            "right_outside_brake_light", "right_inside_brake_light", "right_reverse_light",
            "right_rear_signal", "right_outside_headlight", "right_middle_headlight", 
            "right_inside_headlight", "right_front_signal"
        ]
        led_commands.values = [False] * 8  # Turn off all left part lights
        self.led_publisher_.publish(led_commands)


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
        marker.scale.x = 0.05  # Line width
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for i in range(self.waypointSequence_tf.shape[1]):
            p = Point()
            p.x = float(self.waypointSequence_tf[0, i])
            p.y = float(self.waypointSequence_tf[1, i])
            p.z = 0.0
            marker.points.append(p)
        self.marker_pub.publish(marker)

    def update_pose_from_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())

            # Extract position and orientation from transform
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
    def detection_callback(self, msg):
        self.detection_labels = []

        for det in msg.data.split(', '):
            try:
                label, conf = det.split(':')
                self.detection_labels.append((label.upper(), float(conf)))
            except ValueError:
                self.get_logger().warn(f"Malformed detection: {det}")
                continue
    def get_filtered_detections(self, target_label, min_conf=0.8):
        return [
            (label, conf)
            for label, conf in self.detection_labels
            if label == target_label and conf >= min_conf
        ]
    def is_round_detected(self):
        return len(self.get_filtered_detections('ROUND', 0.92)) > 0

    def cone_detected(self):
        return len(self.get_filtered_detections('CONE', 0.8)) > 0

    def is_person_detected(self):
        return len(self.get_filtered_detections('PERSON', 0.7)) > 0

    def reached_cross_section(self):
        red_detects = self.get_filtered_detections('RED', 0.8)
        green_detects = self.get_filtered_detections('GREEN', 0.7)

        if self.cross_waiting:
            if len(green_detects) > 0:
                self.get_logger().info("GREEN light detected — proceeding.")
                self.cross_waiting = False
                self.cross_cooldown = True
                self.last_cross_cooldown_time = time.time()
                return False
            else:
                self.get_logger().info("Waiting for GREEN light.")
                return True  # Keep waiting for the green light
        # Cooldown after red light
        if self.cross_cooldown:
            if len(red_detects) == 0 or (time.time() - self.last_cross_cooldown_time > 10.0):
                self.get_logger().info("Cooldown over. Ready to proceed.")
                self.cross_cooldown = False
            return False

        # New RED detection logic
        if len(red_detects) > 0 and not self.cross_waiting and self.flag_d:
            self.get_logger().info("RED light detected — waiting for GREEN.")
            self.cross_waiting = True
            self.last_cross_time = time.time()
            return True

        return False

    def reached_stop_sign(self):
        stop_detects = self.get_filtered_detections('STOP', 0.9)
        now = time.time()

        # If we're stopping now
        if self.stop_waiting:
            if now - self.last_stop_time < 3.0:
                return True
            else:
                self.get_logger().info(f"Finished STOP #{self.stop_index}.")
                self.stop_waiting = False
                self.stop_cooldown = True
                self.last_stop_detect_time = now
                return False

        # Cooldown — wait for STOP to go away or 5 seconds max
        if self.stop_cooldown:
            # if len(stop_detects) == 0 or (now - self.last_stop_detect_time > 15.0):
            if (now - self.last_stop_detect_time > 15.0):
                self.get_logger().info("Cooldown over. Looking for new STOP.")
                self.stop_cooldown = False
            return False

        # If STOP detected and not cooling down
        if len(stop_detects) > 0 and not self.stop_waiting:
            self.get_logger().info(f"STOP sign #{self.stop_index + 1} detected. Stopping...")
            self.stop_waiting = True
            self.last_stop_time = now
            self.stop_index += 1
            return True

        return False

    def reached_drop_off_or_traffic_light(self):
        # Check if we are at the drop-off location
        dist_to_drop_off = np.linalg.norm([self.position[0] - self.drop_off[0], self.position[1] - self.drop_off[1]])

        # Check if the traffic light is red
        red_detects = self.get_filtered_detections('RED', 0.83)

        if dist_to_drop_off <= self.goal_tresh or len(red_detects) > 0:
            # If we are at the drop-off or the red light is detected, stop the car
            if not self.stopped:
                self.stopped = True
                self.stop_time = time.time()  # Record the time when the stop happened
                self.get_logger().info("Stopping at drop-off location or red light.")
            return True

        # Reset the stop condition if we've passed the stop point
        if self.stopped and (time.time() - self.stop_time < 3):  # Wait for a while before allowing movement again
            self.stopped = False
            self.get_logger().info("Resuming after stop.")
        return False
    def reached_cross_section(self):
        # Get filtered detections for red and green lights
        red_detects = self.get_filtered_detections('RED', 0.87)
        green_detects = self.get_filtered_detections('GREEN', 0.85)

        if self.cross_waiting:
            if len(green_detects) > 0:
                self.get_logger().info("GREEN light detected — proceeding.")
                self.cross_waiting = False  # Stop waiting for green light
                self.cross_cooldown = True  # Start cooldown after red-green cycle
                self.last_cross_cooldown_time = time.time()
                return False  # Proceed after green light

            else:
                self.get_logger().info("Waiting for green light...")
                return True  # Keep waiting until green light is detected

        # Cooldown after red-green cycle to avoid immediate red light detection
        if self.cross_cooldown:
            if len(red_detects) == 0 or (time.time() - self.last_cross_cooldown_time > 5.0):
                self.get_logger().info("Cross-section cooldown complete.")
                self.cross_cooldown = False  # End cooldown after sufficient time
            return False  # Don't process until cooldown is complete

        # If a red light is detected and we are not already waiting or cooling down
        if len(red_detects) > 0 and not self.cross_waiting:
            self.get_logger().info("RED light detected — waiting for GREEN.")
            self.cross_waiting = True  # Start waiting for green light
            self.last_cross_time = time.time()
            return True  # Continue waiting for green light

        return False  # Default case: Proceed without waiting if no conditions matched
    def publish_commands(self, steering_angle, throttle):
        msg = MotorCommands()
        msg.motor_names = ["steering_angle", "motor_throttle"]
        msg.values = [float(steering_angle), float(throttle)]
        self.motor_command_publisher.publish(msg)

    def control_loop(self):
        tp = self.t
        self.t = time.time() - self.t0
        self.dt = self.t-tp
        self.curr_time = time.time()
        self.update_pose_from_tf()

        x, y, th = self.position[0], self.position[1], self.yaw
        p = ( np.array([x, y])
                    + np.array([np.cos(th), np.sin(th)]) * 0.2)
        v = self.v

        if self.t < self.start_delay:
            self.set_led_color(1)
            self.throttle = 0
            self.delta = 0

        else:
            dist_pick_up = np.linalg.norm([x - self.pick_up[0], y - self.pick_up[1]])
            dist_drop_off = np.linalg.norm([x - self.drop_off[0], y - self.drop_off[1]])
            
            if dist_pick_up <= self.goal_tresh:
                if not self.flag_p and self.is_person_detected() and self.pick_up_index %2 == 0:
                    self.stop_init_p = time.time()
                    self.pick_up_index +=1
                self.flag_p = True
            if dist_drop_off <= self.goal_tresh:
                if not self.flag_d or not self.stopped:
                    self.stop_init_d =time.time()
                    self.stop_time = time.time()
                self.flag_d = True
            if ((self.flag_p or self.flag_d)  and (self.curr_time - self.stop_init_p)<=3) or (self.curr_time - self.stop_init_d)<=3  :
                self.set_led_color(2)
                self.throttle = 0
                self.delta = 0   

            elif self.reached_cross_section():
                self.set_led_color(3)
                self.throttle = 0
                self.delta = 0
            elif self.reached_drop_off_or_traffic_light() :
                self.set_led_color(4)
                self.throttle = 0
                self.delta = 0
                return
            elif self.reached_stop_sign() :
                self.set_led_color(0)
                self.throttle = 0
                self.delta = 0     
            else:
                self.flag_p = False
                self.flag_d = False
                self.flag_pedastrian = False
                self.reached_t = False
                self.set_led_color(1)
                # Speed control
                self.throttle = self.speedController.update(v, self.v_ref, self.dt)

                # Steering control
                self.delta = self.steeringController.update(p, th, v)

            if self.is_round_detected() and not self.round_sign_led_blinking and not self.right_sign_led_blinking:
                self.round_sign_blink_start_time = time.time()
                self.round_sign_last_toggle_time = time.time()
                self.round_sign_led_blinking = True
                self.round_sign_led_on = False

            if self.round_sign_led_blinking:
                elapsed = time.time() - self.round_sign_blink_start_time
                if elapsed > 3.0:
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
                if elapsed > 2.0:
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

def main(args=None):
    rclpy.init(args=args)
    node = VehicleControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()  