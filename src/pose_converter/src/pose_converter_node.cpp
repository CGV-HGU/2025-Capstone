#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "omo_r1_interfaces/srv/reset_odom.hpp"  // reset_odom 서비스 헤더 포함
#include "omo_r1_interfaces/srv/onoff.hpp"       // set_tf_only_mode 서비스 헤더 포함

class PoseConverter : public rclcpp::Node {
public:
  PoseConverter() : Node("pose_converter") {
    // /run_slam/camera_pose 구독: 최신 odometry 데이터를 받아 transform에 사용
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/run_slam/camera_pose", 10,
      std::bind(&PoseConverter::pose_callback, this, std::placeholders::_1));
    
    // /run_slam/tracking_status 구독: tracking 상태 변화(리로컬라이제이션 이벤트)를 감지
    tracking_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/run_slam/tracking_status", 10,
      std::bind(&PoseConverter::tracking_callback, this, std::placeholders::_1));
    
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PoseConverter::publish_transform, this));
    
    latest_pose_available_ = false;
    prev_tracking_ = false;
    
    // 서비스 클라이언트 생성
    reset_odom_client_ = this->create_client<omo_r1_interfaces::srv::ResetOdom>("reset_odom");
    set_tf_only_mode_client_ = this->create_client<omo_r1_interfaces::srv::Onoff>("set_tf_only_mode");
  }

private:
  // Odometry 메시지를 받으면 최신 pose를 저장
  void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_pose_ = *msg;
    latest_pose_available_ = true;
  }

  // 주기적으로 tf 변환을 브로드캐스트
  void publish_transform() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";

    if (latest_pose_available_) {
      t.transform.translation.x = latest_pose_.pose.pose.position.x * 10.0;
      t.transform.translation.y = latest_pose_.pose.pose.position.y * 10.0;
      t.transform.translation.z = 0.0;
      t.transform.rotation.x = latest_pose_.pose.pose.orientation.x;
      t.transform.rotation.y = latest_pose_.pose.pose.orientation.y * -1;
      t.transform.rotation.z = latest_pose_.pose.pose.orientation.z;
      t.transform.rotation.w = latest_pose_.pose.pose.orientation.w;
    } else {
      t.transform.translation.x = 0.0;
      t.transform.translation.y = 0.0;
      t.transform.translation.z = 0.0;
      t.transform.rotation.x = 0.0;
      t.transform.rotation.y = 0.0;
      t.transform.rotation.z = 0.0;
      t.transform.rotation.w = 1.0;
    }

    tf_broadcaster_->sendTransform(t);
  }

  // tracking 상태 변화 콜백: false -> true (리로컬라이제이션 이벤트)일 때 reset_odom과 set_tf_only_mode 호출
  void tracking_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    bool current_tracking = msg->data;
    if (!prev_tracking_ && current_tracking) {
      RCLCPP_INFO(this->get_logger(), "Relocalization has occurred.");
    
      // reset_odom 호출
      call_reset_odom();
      
      // set_tf_only_mode를 true로 재설정
      call_set_tf_only_mode(true);
    }
    else if (prev_tracking_ && !current_tracking) {
      // set_tf_only_mode를 true로 재설정
      call_set_tf_only_mode(false);
    }
    prev_tracking_ = current_tracking;
  }

  // reset_odom 서비스를 호출하여 오도메트리 초기화 (x=0, y=0, theta=0)
  void call_reset_odom() {
    if (!reset_odom_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "reset_odom service not available");
      return;
    }
    auto request = std::make_shared<omo_r1_interfaces::srv::ResetOdom::Request>();
    request->x = 0.0;
    request->y = 0.0;
    request->theta = 0.0;
    RCLCPP_INFO(this->get_logger(), "Calling reset_odom service with x=0, y=0, theta=0");
    auto result_future = reset_odom_client_->async_send_request(request);
  }

  // set_tf_only_mode 서비스를 호출하여 TF-Only 모드 활성화/비활성화
  void call_set_tf_only_mode(bool enable) {
    if (!set_tf_only_mode_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "set_tf_only_mode service not available");
      return;
    }
    auto request = std::make_shared<omo_r1_interfaces::srv::Onoff::Request>();
    request->set = enable;
    RCLCPP_INFO(this->get_logger(), "Calling set_tf_only_mode service with value: %s", enable ? "true" : "false");
    auto result_future = set_tf_only_mode_client_->async_send_request(request);
  }

  // 멤버 변수 선언
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tracking_subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry latest_pose_;
  bool latest_pose_available_;
  bool prev_tracking_;
  rclcpp::Client<omo_r1_interfaces::srv::ResetOdom>::SharedPtr reset_odom_client_;
  rclcpp::Client<omo_r1_interfaces::srv::Onoff>::SharedPtr set_tf_only_mode_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseConverter>());
  rclcpp::shutdown();
  return 0;
}
