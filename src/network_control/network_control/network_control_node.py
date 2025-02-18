import rclpy
import time
import json
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from omo_r1_interfaces.srv import Battery
from tf2_ros import Buffer, TransformListener
from .supabase_manager import SupabaseManager

class NetworkControl(Node):
    def __init__(self):
        super().__init__('network_control')
        self.supabase_manager = SupabaseManager()
        self.battery_client = self.create_client(Battery, 'check_battery')
        self.nav_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # TF2 Transform Listener 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_battery_status(self):
        """배터리 상태를 조회하여 반환"""
        if not self.battery_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Battery service not available")
            return None
        req = Battery.Request()
        future = self.battery_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            return future.result().soc  # 배터리 잔량 반환
        return None

    def get_robot_position(self):
        """TF2를 이용하여 로봇의 현재 위치 조회"""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return [transform.transform.translation.x, transform.transform.translation.y]
        except Exception as e:
            self.get_logger().error(f"Failed to get robot position: {e}")
            return [0.0, 0.0]  # 기본값 반환

    def publish_goal(self, goal_position):
        """Nav2에 목표 위치 전달"""
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = goal_position[0]
        goal_msg.pose.position.y = goal_position[1]
        goal_msg.pose.position.z = 0.0 

        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0  
        goal_msg.pose.orientation.w = 1.0  # heading은 0
        self.nav_goal_pub.publish(goal_msg)
        self.get_logger().info(f"Sent goal pose: {goal_position}")
        print("published goal")

    def run(self):
        while rclpy.ok():
            battery_status = self.get_battery_status()
            if battery_status is None:
                continue
            
            # 로봇 위치 가져오기
            robot_position = self.get_robot_position()

            # 로봇 상태 업데이트
            robot_status = {
                "id": int(self.supabase_manager.robot_id),
                "status": 1,  # 정상 상태
                "battery": int(battery_status),
                "position": robot_position
            }
            self.supabase_manager.update_robot_status(robot_status)
            
            # 새로운 요청 확인
            request = self.supabase_manager.fetch_request()
            if request and "goal_position" in request:
                print("requesting")
                goal_position = request["goal_position"]
                self.publish_goal(goal_position)
                
                # 응답 데이터 생성 (예제: 생성된 경로를 가정)
                response_data = {
                    "robot_id": self.supabase_manager.robot_id,
                    "complete": False,
                    "generated_path": [goal_position]  # 실제 경로를 받아야 함
                }
                self.supabase_manager.create_response(response_data)
                self.get_logger().info(f"Response created for goal {goal_position}")
            
            time.sleep(1)  # 1초 주기로 실행

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