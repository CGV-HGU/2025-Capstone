#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import numpy as np

class FloorDetector(Node):
    def __init__(self):
        super().__init__('floor_detector')

        # Publisher for the OccupancyGrid map that will be visualized in RViz
        self.map_pub = self.create_publisher(OccupancyGrid, '/segmented_map', 10)

        # Subscriber for raw camera images
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)

        # YOLO model initialization
        model_path = "/home/caleb/ros2_ws/src/freespace_detection/scripts/best.pt"
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            rclpy.shutdown()
            return

        # Bridge to convert ROS images to OpenCV images
        self.bridge = CvBridge()
        self.get_logger().info("Floor Detector Node initialized.")

    def image_callback(self, data):
        try:
            # Convert ROS Image message to an OpenCV image
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error converting image: {e}")
            return

        # Run the YOLO model to obtain segmentation results
        results = self.model(frame, stream=True, conf=0.5)

        # Create a blank mask same size as the incoming frame
        segmented_mask = np.zeros(frame.shape[:2], dtype=np.uint8)

        # Process the segmentation results: for each detected mask, assign segmented regions with 255
        for result in results:
            if result.masks is not None:
                masks = result.masks.data.cpu().numpy()  # Each mask is a 2D array
                for mask in masks:
                    segmented_mask[mask > 0] = 255

        # Convert the segmentation mask into occupancy grid values:
        # In a nav_msgs/OccupancyGrid, cells are:
        #  -100 or 0 typically indicate free space,
        # 100 indicates occupied and -1 indicates unknown.
        # Here, we mark segmented (detected floor/object) areas as occupied.
        # Flip vertically because OccupancyGrid expects data starting at the bottom-left.
        occupancy_mask = np.flipud(segmented_mask)
        occupancy_data = (occupancy_mask > 0).astype(np.int8) * 100

        # Create an OccupancyGrid message
        occ_grid = OccupancyGrid()
        occ_grid.header.stamp = self.get_clock().now().to_msg()
        occ_grid.header.frame_id = "map"  # RViz will need to use the same frame

        # Set metadata for the grid (using arbitrarily chosen values; adjust as needed)
        resolution = 0.05  # meters per cell; change as appropriate for your scenario
        occ_grid.info.resolution = resolution
        height, width = occupancy_data.shape
        occ_grid.info.width = width
        occ_grid.info.height = height

        # Define the origin of the map (bottom-left of the map is set at (0,0))
        occ_grid.info.origin.position.x = 0.0
        occ_grid.info.origin.position.y = 0.0
        occ_grid.info.origin.position.z = 0.0
        occ_grid.info.origin.orientation.w = 1.0

        # Flatten the occupancy data into a one-dimensional list (row-major order)
        occ_grid.data = occupancy_data.flatten().tolist()

        # Publish the occupancy grid so that it can be visualized in RViz as a 2D map
        self.map_pub.publish(occ_grid)

        # Optionally, display the segmentation result locally for debugging
        cv2.imshow("Segmentation Result", segmented_mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    floor_detector = FloorDetector()
    try:
        rclpy.spin(floor_detector)
    except KeyboardInterrupt:
        pass
    finally:
        floor_detector.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
