#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import numpy as np

class floorDetector(Node):
    def __init__(self):
        super().__init__('floor_detector')

        # Publisher
        self.roi_pub = self.create_publisher(Bool, '/floor_detector', 10)

        # Subscriber
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)

        # YOLO model
        model_path = "/home/cgv/ros2_ws/src/freespace_detection/scripts/best.pt"
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            rclpy.shutdown()
            return

        # ROI Definition
        self.roi_x_min, self.roi_y_min, self.roi_x_max, self.roi_y_max = 140, 240, 180, 250
        self.width = self.roi_x_max - self.roi_x_min
        self.height = self.roi_y_max - self.roi_y_min
        self.roi_area = self.width * self.height
        self.get_logger().info(f"ROI defined: x=({self.roi_x_min}, {self.roi_x_max}), "
                               f"y=({self.roi_y_min}, {self.roi_y_max}), area={self.roi_area}")
        # CV Bridge for converting ROS images to OpenCV
        self.bridge = CvBridge()

        self.get_logger().info("Floor Detector Node initialized.")

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Run YOLO model on the frame
        results = self.model(frame, stream=True, conf=0.5)

        in_roi_80_percent = False  # 기본값

        # Process results
        for result in results:
            if result.masks is not None:
                masks = result.masks.data.cpu().numpy()

                for mask in masks:
                    y_indices, x_indices = np.where(mask)

                    # ROI 내 mask 픽셀 개수 계산
                    within_roi = ((x_indices >= self.roi_x_min) & (x_indices <= self.roi_x_max) &
                                  (y_indices >= self.roi_y_min) & (y_indices <= self.roi_y_max))
                    mask_area = within_roi.sum()

                    # ROI 영역의 80% 이상이 마스크에 포함되면 감지로 판단
                    if mask_area >= self.roi_area * 0.8:
                        in_roi_80_percent = True
                        break

        # 결과를 퍼블리시
        self.roi_pub.publish(Bool(data=in_roi_80_percent))

        # ROI 사각형과 감지 결과 텍스트를 이미지에 표시
        color = (0, 255, 0) if in_roi_80_percent else (0, 0, 255)
        cv2.rectangle(frame, (self.roi_x_min, self.roi_y_min), (self.roi_x_max, self.roi_y_max), color, 2)
        text = "Floor Detected" if in_roi_80_percent else "Floor Not Detected"
        cv2.putText(frame, text, (self.roi_x_min, self.roi_y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # 결과 이미지 창에 표시
        cv2.imshow("Detection Result", frame)
        cv2.waitKey(1)  # 창 업데이트를 위한 대기

def main(args=None):
    rclpy.init(args=args)
    floor_detector = floorDetector()
    try:
        rclpy.spin(floor_detector)
    except KeyboardInterrupt:
        pass
    finally:
        floor_detector.destroy_node()
        cv2.destroyAllWindows()  # 모든 OpenCV 창 종료
        rclpy.shutdown()

if __name__ == '__main__':
    main()

