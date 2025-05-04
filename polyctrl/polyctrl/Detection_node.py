#!/usr/bin/env python3
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import yaml
from ultralytics import YOLO  # YOLOv8
import os
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
DETECTION_CLASSES = ['cone','green', 'red', 'yellow', 'stop', 'yield', 'round', 'person']

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/color_image',
            self.image_callback,
            qos_profile_sensor_data
            )
        self.detection_pub = self.create_publisher(String, 'yolo_det', 10)
        model_path = '/workspaces/isaac_ros-dev/ros2/src/polyctrl/polyctrl/best.pt'  
        self.model = self.load_model(model_path)

        module_path = '/workspaces/isaac_ros-dev/ros2/src/polyctrl/polyctrl/data.yaml'  
        self.class_names = self.get_class_names_from_module(module_path)

    def load_model(self, model_path):
        return YOLO(model_path)

    def get_class_names_from_module(self, module_path):
        with open(module_path, 'r') as file:
            data = yaml.safe_load(file)
        class_names = data.get('names', [])
        return class_names

    def image_callback(self, msg):
        try:
            if not msg.data:
                self.get_logger().warn("Received empty image data.")
                return

            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if frame is None:
                self.get_logger().warn("cv2 conversion returned None.")
                return

        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        # Detect and visualize
        annotated_frame, detections = self.detect_objects(frame)
        cv2.imshow("YOLOv8 Detection", annotated_frame)
        cv2.waitKey(1)

        if detections:
            det_strings = [f"{label}:{conf:.2f}" for label, conf in detections]
            det_msg = String()
            det_msg.data = ', '.join(det_strings)
            self.detection_pub.publish(det_msg)
            self.get_logger().info(f"Published detection: {det_msg.data}")

    def detect_objects(self, frame):
        detections = []
        results = self.model.predict(frame, save=False, conf=0.6, device='cuda')[0]
        annotated_frame = frame.copy()

        for result in results.boxes:
            cls_id = int(result.cls[0])
            confidence = float(result.conf[0])

            if cls_id < len(self.class_names):
                label = self.class_names[cls_id].lower()
                if label in DETECTION_CLASSES:
                    detections.append((label.upper(), confidence))

                    box = result.xyxy[0].tolist()
                    x1, y1, x2, y2 = map(int, box)
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    cv2.putText(annotated_frame, f"{label}: {confidence:.2f}",
                                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 0, 255), 2)

        return annotated_frame, detections


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # mode of use example
    main()

