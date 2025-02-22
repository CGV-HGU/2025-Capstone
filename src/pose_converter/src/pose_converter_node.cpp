#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/parameter_client.hpp>  // 수정: async_parameters_client.hpp 대신 parameter_client.hpp 사용
#include "omo_r1_interfaces/srv/reset_odom.hpp"  // reset_odom 서비스 헤더 포함
#include "omo_r1_interfaces/srv/onoff.hpp"       // set_tf_only_mode 서비스 헤더 포함

using namespace std::chrono_literals;

class PoseConverter : public rclcpp::Node {
public:
  PoseConverter() : Node("pose_converter") {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/run_slam/camera_pose", 10,
      std::bind(&PoseConverter::pose_callback, this, std::placeholders::_1));

    tracking_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/run_slam/tracking_status", 10,
      std::bind(&PoseConverter::tracking_callback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&PoseConverter::publish_transform, this));

    param_timer_ = this->create_wall_timer(
      500ms,
      std::bind(&PoseConverter::update_scale_factor, this));

    latest_pose_available_ = false;
    prev_tracking_ = false;
    current_tracking = false;

    reset_odom_client_ = this->create_client<omo_r1_interfaces::srv::ResetOdom>("reset_odom");
    set_tf_only_mode_client_ = this->create_client<omo_r1_interfaces::srv::Onoff>("set_tf_only_mode");

    // /omo_r1_motor_setting 노드의 파라미터 클라이언트 생성
    param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/omo_r1_motor_setting");
  }

private:
  void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_pose_ = *msg;
    latest_pose_available_ = true;
  }

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

    if(current_tracking) {
      call_reset_odom();
    }

    tf_broadcaster_->sendTransform(t);
  }

  void tracking_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    current_tracking = msg->data;
  }

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

  // 0.5초마다 odom->base_footprint의 scale_factor 파라미터를 설정
  void update_scale_factor() {

    //되는거 확인했고 이제 scale_factor 계산 필요
    if (!param_client_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "Parameter service on /omo_r1_motor_setting not available");
      return;
    }
    std::vector<rclcpp::Parameter> parameters;
    parameters.push_back(rclcpp::Parameter("scale_factor", 1.0));
    auto future = param_client_->set_parameters(parameters);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tracking_subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr param_timer_;
  nav_msgs::msg::Odometry latest_pose_;
  bool latest_pose_available_;
  bool prev_tracking_;
  bool current_tracking;
  rclcpp::Client<omo_r1_interfaces::srv::ResetOdom>::SharedPtr reset_odom_client_;
  rclcpp::Client<omo_r1_interfaces::srv::Onoff>::SharedPtr set_tf_only_mode_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseConverter>());
  rclcpp::shutdown();
  return 0;
}
