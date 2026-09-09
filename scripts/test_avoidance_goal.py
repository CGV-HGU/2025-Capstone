#!/usr/bin/env python3
import sys
import time
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
import math

class AvoidanceGoalSender(Node):
    def __init__(self, forward_dist=5.0):
        super().__init__('avoidance_goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.forward_dist = forward_dist

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def initialize_and_send_goal(self):
        # 1) Publish initial pose to map (0, 0, 0)
        self.get_logger().info('Initializing robot pose at (0, 0, 0)...')
        init_msg = PoseWithCovarianceStamped()
        init_msg.header.frame_id = 'map'
        init_msg.header.stamp = self.get_clock().now().to_msg()
        init_msg.pose.pose.position.x = 0.0
        init_msg.pose.pose.position.y = 0.0
        init_msg.pose.pose.orientation.w = 1.0
        init_msg.pose.covariance[0] = 0.25
        init_msg.pose.covariance[7] = 0.25
        init_msg.pose.covariance[35] = 0.06

        for _ in range(3):
            self.init_pose_pub.publish(init_msg)
            rclpy.spin_once(self, timeout_sec=0.2)

        # 2) Wait for action server
        self.get_logger().info('Waiting for /navigate_to_pose action server...')
        t_start = time.time()
        while not self._action_client.wait_for_server(timeout_sec=0.5):
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - t_start > 10.0:
                self.get_logger().error('Nav2 Action server not available. Is navigation2 running?')
                return False

        # 3) Spin to fill TF buffer
        t_start = time.time()
        while time.time() - t_start < 1.5:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Current robot pose lookup
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time()
            )
            cur_x = t.transform.translation.x
            cur_y = t.transform.translation.y
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            self.get_logger().info(f'Current robot pose: x={cur_x:.2f}, y={cur_y:.2f}, yaw={math.degrees(yaw):.1f} deg')
        except Exception as e:
            self.get_logger().warn(f'TF lookup: {e}, using origin (0, 0).')
            cur_x, cur_y, yaw = 0.0, 0.0, 0.0

        target_x = cur_x + self.forward_dist * math.cos(yaw)
        target_y = cur_y + self.forward_dist * math.sin(yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Sending Nav2 Goal: Forward {self.forward_dist:.1f}m -> Target (x={target_x:.2f}, y={target_y:.2f})'
        )

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2. Please ensure Nav2 lifecycle is active.')
            return

        self.get_logger().info('Goal accepted! Robot moving forward. Avoidance active...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        dist_remain = feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {dist_remain:.2f}m', throttle_duration_sec=1.0)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('Navigation Succeeded! Reached target position.')
        else:
            self.get_logger().warn(f'Navigation ended with status: {status}')
        rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser(description='Send relative forward navigation goal for avoidance test.')
    parser.add_argument('-d', '--distance', type=float, default=5.0, help='Forward distance in meters (default: 5.0m)')
    args, _ = parser.parse_known_args()

    rclpy.init()
    sender = AvoidanceGoalSender(forward_dist=args.distance)
    if sender.initialize_and_send_goal():
        try:
            rclpy.spin(sender)
        except KeyboardInterrupt:
            pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
