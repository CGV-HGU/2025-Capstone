# network_control/network_control.py
# 2025 CGV Capstone
# Author: 22100600 이현서 <hslee@handong.ac.kr>

import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from omo_r1_interfaces.srv import Battery
from tf2_ros import Buffer, TransformListener
from .supabase_manager import SupabaseManager
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class NetworkControl(Node):
    def __init__(self):
        super().__init__('network_control')
        self.supabase_manager = SupabaseManager()
        self.battery_client = self.create_client(Battery, 'check_battery')
        self.nav_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_pub   = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_goal = None
        self.last_request_id = None                   # 2. 중복 요청 방지용
        self.robot_status = 0
        self.has_arrived = False                       # 8. 도착 플래그
        self.latest_path = None

        self.path_subscriber = self.create_subscription(Path, '/plan', self.path_callback, 1)
        self.navigate_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.current_goal_handle = None
        self.last_publish_response_id = None           # 3. publish 시 response ID 저장
        self.last_arrival_response_id = None           # 3. arrival 시 response ID 저장

    def path_callback(self, msg):
        self.latest_path = msg
        self.get_logger().info("Received new global plan from /plan topic")

    def get_battery_status(self):
        try:
            if not self.battery_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("Battery service not available, retrying...")
                return None
            req = Battery.Request()
            future = self.battery_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            return future.result().soc if future.result() else None
        except Exception as e:
            self.get_logger().error(f"Battery check error: {e}")
            return None

    def get_robot_position(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return [tf.transform.translation.x, tf.transform.translation.y]
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return [0.0, 0.0]

    def publish_and_log_goal(self, goal_position, request_id):
        # publish goal
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = goal_position
        msg.pose.orientation.w = 1.0
        self.nav_goal_pub.publish(msg)
        self.get_logger().info(f"Sent goal pose: {goal_position}")

        # log to DB as incomplete response
        path = []
        if self.latest_path:
            path = [[p.pose.position.x, p.pose.position.y] for p in self.latest_path.poses]
            self.latest_path = None          # 7. 경로 클리어
        try:
            response_id = self.supabase_manager.create_response({
                'complete': False,
                'generated_path': path
            })
            self.last_publish_response_id = response_id
        except Exception as e:
            self.get_logger().error(f"Failed to log publish-response: {e}")

    def _result_callback(self, future):
        status = future.result().status
        # 4. 완료 시 handle 리셋
        self.current_goal_handle = None
        if status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info("Goal reached successfully!")
            self.has_arrived = True
            self.robot_status = 0

            # log arrival as new response entry
            try:
                arrival_id = self.supabase_manager.create_response({
                    'complete': True,
                    'generated_path': []
                })
                self.last_arrival_response_id = arrival_id
            except Exception as e:
                self.get_logger().error(f"Failed to log arrival-response: {e}")

            # 7. 경로 클리어 & has_arrived 활용
            self.latest_path = None
        else:
            self.get_logger().info(f"Goal ended with status {status}")

    def _goal_response_callback(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn("Goal rejected by action server.")
            return
        self.get_logger().info("Goal accepted.")
        self.current_goal_handle = gh
        gh.get_result_async().add_done_callback(self._result_callback)

    def send_nav_goal(self, goal_position):
        try:
            if not self.navigate_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("NavigateToPose server not available, retry soon.")
                return False
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = goal_position
            goal.pose.pose.orientation.w = 1.0
            future = self.navigate_client.send_goal_async(goal)
            future.add_done_callback(self._goal_response_callback)
            self.get_logger().info(f"Sent goal to Nav2: {goal_position}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to send nav goal: {e}")
            return False

    def emergency_stop(self):
        # 5. 속도 0 명령 퍼블리시
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Emergency stop: cmd_vel zeroed.")

        # cancel action
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.get_logger().info("Emergency stop: nav goal cancelled.")
            self.current_goal_handle = None  # 4. handle 리셋
        self.robot_status = 1

    def run(self):
        while rclpy.ok():
            bat = self.get_battery_status()
            if bat is None:
                time.sleep(1)
                continue

            pos = self.get_robot_position()
            # 10. 예외 안전 처리
            try:
                self.supabase_manager.update_robot_status({
                    'status': self.robot_status,
                    'battery': int(bat),
                    'position': pos
                })
            except Exception as e:
                self.get_logger().error(f"Status update error: {e}")

            req = self.supabase_manager.fetch_request()
            if req:
                req_id = req.get('id')
                # 2. 중복 요청 방지
                if req_id == self.last_request_id:
                    time.sleep(1)
                    continue
                self.last_request_id = req_id

                if req.get('emergency_stop', False):
                    self.emergency_stop()
                elif 'goal_position' in req:
                    gp = req['goal_position']
                    # 새 목표가 기존과 같아도 요청별로 처리
                    self.publish_and_log_goal(gp, req_id)
                    sent = self.send_nav_goal(gp)
                    if sent:
                        self.robot_status = 3
                        self.has_arrived = False  # 8. 도착 플래그 리셋
                        self.last_goal = gp
            time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    node = NetworkControl()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
