#!/usr/bin/env python3
import sys
import time
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
import math

class AvoidanceGoalSender(Node):
    def __init__(self, rel_x=10.0, rel_y=0.0, rel_yaw_deg=0.0, timeout_sec=60.0):
        super().__init__('avoidance_goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.rel_x = rel_x
        self.rel_y = rel_y
        self.rel_yaw_rad = math.radians(rel_yaw_deg)
        self.timeout_sec = timeout_sec
        self.timeout_timer = None
        self.goal_handle = None

        # 순수 주행 정밀 측정을 위한 변수
        self.t_script_start = time.time()
        self.t_motion_start = None
        self.t_motion_end = None
        self.motion_started = False
        self.motion_ended = False
        self.start_pose = None
        self.max_x = 0.0
        self.max_y_deviation = 0.0
        self.max_v = 0.0
        self.max_w = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def initialize_and_send_goal(self):
        # 1) Publish initial pose to map (0, 0, 0)
        self.get_logger().info('Checking robot pose initialization...')
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
            rclpy.spin_once(self, timeout_sec=0.1)

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
        while time.time() - t_start < 1.0:
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

        # Relative transformation to map frame
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        target_x = cur_x + self.rel_x * cos_yaw - self.rel_y * sin_yaw
        target_y = cur_y + self.rel_x * sin_yaw + self.rel_y * cos_yaw
        target_yaw = yaw + self.rel_yaw_rad

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(target_yaw / 2.0)

        self.get_logger().info(
            f'Sending Nav2 Goal: Relative (dx={self.rel_x:.2f}m, dy={self.rel_y:.2f}m, d_yaw={math.degrees(self.rel_yaw_rad):.1f} deg) '
            f'-> Target Map Pose (x={target_x:.2f}, y={target_y:.2f}, yaw={math.degrees(target_yaw):.1f} deg)'
        )

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = abs(msg.twist.twist.angular.z)
        v = math.sqrt(vx * vx + vy * vy)

        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y

        if self.start_pose is None:
            self.start_pose = (px, py)

        dx = px - self.start_pose[0]
        dy = abs(py - self.start_pose[1])
        dist_from_start = math.sqrt(dx * dx + dy * dy)

        # 실제 바퀴가 구르기 시작한 순간(v > 0.02 m/s 또는 이동 3cm 이상)을 정밀 포착
        if not self.motion_started and self.goal_handle is not None and (v > 0.02 or dist_from_start > 0.03):
            self.motion_started = True
            self.t_motion_start = time.time()
            self.get_logger().info('🚀 로봇 실제 주행 시작 감지! [순수 주행 시간 정밀 타이머 START]')

        if self.motion_started and not self.motion_ended:
            self.max_v = max(self.max_v, v)
            self.max_w = max(self.max_w, wz)
            self.max_x = max(self.max_x, dx)
            self.max_y_deviation = max(self.max_y_deviation, dy)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2. Please ensure Nav2 lifecycle is active.')
            rclpy.shutdown()
            return

        self.get_logger().info(f'Goal accepted! Waiting for robot motion to start pure timer (Timeout: {self.timeout_sec:.1f}s)...')
        self.timeout_timer = self.create_timer(self.timeout_sec, self.timeout_callback)
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def timeout_callback(self):
        self.motion_ended = True
        self.t_motion_end = time.time()
        self.get_logger().error(f'Navigation TIMED OUT after {self.timeout_sec:.1f}s! Robot stuck or unable to reach goal.')
        if self.timeout_timer:
            self.timeout_timer.cancel()
        if self.goal_handle:
            self.get_logger().info('Canceling active goal...')
            self.goal_handle.cancel_goal_async()
        self.print_summary(outcome_str=f"❌ 타임아웃 중도 정지 ({self.timeout_sec:.1f}s 초과)")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        dist_remain = feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {dist_remain:.2f}m', throttle_duration_sec=1.0)

    def get_result_callback(self, future):
        self.motion_ended = True
        self.t_motion_end = time.time()
        if self.timeout_timer:
            self.timeout_timer.cancel()
        status = future.result().status
        if status == 4:
            outcome_str = "✅ 목표 도달 완주 성공 (SUCCESS)"
        else:
            outcome_str = f"⚠️ 중도 종료 (Nav2 Status: {status})"
        self.print_summary(outcome_str=outcome_str)
        rclpy.shutdown()

    def print_summary(self, outcome_str=""):
        pure_duration = (self.t_motion_end - self.t_motion_start) if (self.t_motion_start and self.t_motion_end) else 0.0
        init_delay = (self.t_motion_start - self.t_script_start) if (self.t_script_start and self.t_motion_start) else 0.0

        print("\n" + "=" * 65)
        print("🏁 [주행 완료 - 순수 주행 정밀 실측 결과 (Pure Motion Report)]")
        print("=" * 65)
        print(f"⏱️  순수 주행 시간 (Pure Motion Time) : {pure_duration:.2f} 초 (준비 대기 {init_delay:.2f}초 제외)")
        print(f"📏  전진 주행 거리 (Travel X)         : {self.max_x:.2f} m")
        print(f"↔️  최대 회피 이탈폭 (Max Lateral Y)  : {self.max_y_deviation:.3f} m")
        print(f"🏎️  최고 선속도 (Max Linear Speed)   : {self.max_v:.2f} m/s (설정 0.30 m/s)")
        print(f"🔄  최고 각속도 (Max Angular Speed)  : {self.max_w:.3f} rad/s")
        print(f"📌  최종 판정 (Navigation Outcome)   : {outcome_str}")
        print("=" * 65 + "\n")

def main():
    parser = argparse.ArgumentParser(description='Send relative navigation goal (straight or cornering) for avoidance test.')
    parser.add_argument('-d', '--distance', type=float, default=None, help='Straight distance in meters (shortcut for -x)')
    parser.add_argument('-x', type=float, default=10.0, help='Forward distance in meters (default: 10.0m)')
    parser.add_argument('-y', type=float, default=0.0, help='Lateral distance in meters (+: left, -: right, default: 0.0m)')
    parser.add_argument('-a', '--angle', type=float, default=0.0, help='Heading angle in degrees (+: left/CCW, -: right/CW, default: 0.0)')
    parser.add_argument('--corner', choices=['left', 'right'], default=None, help='Preset corner turn (e.g. --corner left sets x=4.0, y=3.0, a=90)')

    parser.add_argument('-t', '--timeout', type=float, default=60.0, help='Navigation timeout in seconds (default: 60.0s)')

    args, _ = parser.parse_known_args()

    rel_x = args.x
    rel_y = args.y
    rel_angle = args.angle

    if args.distance is not None:
        rel_x = args.distance
        rel_y = 0.0
        rel_angle = 0.0

    if args.corner == 'left':
        rel_x = 4.0
        rel_y = 3.0
        rel_angle = 90.0
    elif args.corner == 'right':
        rel_x = 4.0
        rel_y = -3.0
        rel_angle = -90.0

    rclpy.init()
    sender = AvoidanceGoalSender(rel_x=rel_x, rel_y=rel_y, rel_yaw_deg=rel_angle, timeout_sec=args.timeout)
    if sender.initialize_and_send_goal():
        try:
            rclpy.spin(sender)
        except KeyboardInterrupt:
            pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
