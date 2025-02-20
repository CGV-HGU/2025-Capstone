import rclpy
import time
import json
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path  # /plan 토픽은 nav_msgs/Path 타입
from omo_r1_interfaces.srv import Battery
from tf2_ros import Buffer, TransformListener
from .supabase_manager import SupabaseManager

#test
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class NetworkControl(Node):
    def __init__(self):
        super().__init__('network_control')
        self.supabase_manager = SupabaseManager()
        self.battery_client = self.create_client(Battery, 'check_battery')
        self.nav_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # TF2 Transform Listener 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 이전 goal_pose를 저장할 변수 (초기값: None)
        self.last_goal = None
        # 로봇 상태
        self.robot_status = 0
        # 목적지 도착여부
        self.has_arrived= False

        # Nav2가 생성한 경로를 저장할 변수
        self.latest_path = None
        # /plan 토픽 구독: Nav2 global planner가 생성한 경로 (nav_msgs/Path)
        self.path_subscriber = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            1
        )

        # test: used to goal checker
        self.navigate_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')


    def path_callback(self, msg):
        """/plan 토픽에서 수신된 경로 메시지를 저장"""
        self.latest_path = msg
        self.get_logger().info("Received new global plan from /plan topic")

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

    #test
    def _result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info("Goal reached successfully!")
            # 도착 성공 시 response의 complete를 True로 업데이트
            if self.last_response_id is not None:
                self.has_arrived = True
                self.robot_status = 0
        else:
            self.get_logger().info(f"Goal ended with status {status}")
            # 실패나 취소인 경우에도 response를 업데이트 (필요시)
            if self.last_response_id is not None:
                self.has_arrived= False

    
    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal was rejected by the action server.")
            return
        
        self.get_logger().info("Goal accepted!")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def send_nav_goal(self, goal_position):
        # 1) 액션 서버가 준비될 때까지 대기
        if not self.navigate_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available!")
            return

        # 2) Goal 메시지 구성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_position[0]
        goal_msg.pose.pose.position.y = goal_position[1]
        goal_msg.pose.pose.orientation.w = 1.0

        # 3) 액션 서버에 비동기로 목표 전송
        send_goal_future = self.navigate_client.send_goal_async(
            goal_msg
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        self.get_logger().info(f"Sent goal to /navigate_to_pose: {goal_position}")


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
                "status": self.robot_status,
                "battery": int(battery_status),
                "position": robot_position
            }
            self.supabase_manager.update_robot_status(robot_status)
            
            # 새로운 요청 확인
            request = self.supabase_manager.fetch_request()
            if request and "goal_position" in request:
                goal_position = request["goal_position"]
                # 이전 goal이 없거나, 현재 요청된 goal이 이전과 다를 때에만 퍼블리시
                if self.last_goal != goal_position:
                    self.publish_goal(goal_position)
                    self.last_goal = goal_position  # 현재 goal 저장

                    # === 액션 기반 목표 전송 ===
                    self.send_nav_goal(goal_position)
                    self.robot_status = 3
                    self.has_arrived = False
                    
                    # Nav2에서 수신한 경로 데이터를 response_data에 포함시키기
                    generated_path = []
                    if self.latest_path:
                        # 경로의 각 Pose에서 (x, y) 좌표를 추출 (필요에 따라 다른 정보를 포함할 수 있음)
                        for pose_stamped in self.latest_path.poses:
                            x = pose_stamped.pose.position.x
                            y = pose_stamped.pose.position.y
                            generated_path.append([x, y])
                    else:
                        self.get_logger().warn("No global plan received from /plan topic")

            # 응답 데이터 생성
            response_data = {
                "robot_id": self.supabase_manager.robot_id,
                "complete": self.has_arrived,
                "generated_path": generated_path
            }
            query_result = self.supabase_manager.create_response(response_data)
            if query_result and query_result.data and "id" in query_result.data[0]:
                self.last_response_id = query_result.data[0]["id"]
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
