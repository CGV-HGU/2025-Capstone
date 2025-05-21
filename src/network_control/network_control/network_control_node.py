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
        initial = self.supabase_manager.fetch_request()
        self.min_request_id = initial['id'] if initial else 0
        self.get_logger().info(f"Ignoring any request id ≤ {self.min_request_id}")

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
                'complete': False
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
                    'complete': True
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

        # If we're in emergency-stop mode, cancel immediately
        if self.robot_status == 1:
            self.get_logger().info("Emergency ongoing at goal-acceptance → cancelling goal")
            gh.cancel_goal_async()
            # Do not register the normal result callback
            return

        # Normal operation: watch for result
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
        self.get_logger().info("Emergency stop triggered!")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

        # Cancel any in-flight goal
        if self.current_goal_handle:
            self.get_logger().info("Cancelling current Nav2 goal...")
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        else:
            self.get_logger().info("No accepted goal yet, will cancel upon acceptance.")

        self.robot_status = 1

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            # … battery check, status update, etc …

            req = self.supabase_manager.fetch_request()
            if not req or req['id'] <= self.min_request_id:
                time.sleep(1)
                continue

            # bump watermark
            self.min_request_id = req['id']
            self.get_logger().info(f"Processing new request {self.min_request_id}")

            # emergency stop: drive==False
            if req.get("drive") is False:
                self.emergency_stop()

            # navigation goal: only if goal_position is not None
            elif req.get("goal_position") is not None:
                gp = req["goal_position"]
                # now guaranteed to be a two‐element list
                self.publish_and_log_goal(gp, self.min_request_id)
                sent = self.send_nav_goal(gp)
                if sent:
                    self.robot_status = 3
                    self.has_arrived = False
                    self.last_goal = gp

            # anything else (e.g. a weird row with drive=True but no goal) we just ignore
            else:
                self.get_logger().warn(f"Ignoring request {self.min_request_id}: no actionable fields")
            
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
