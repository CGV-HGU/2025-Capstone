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
    vslam_pose_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/run_slam/camera_pose", 10,
      std::bind(&PoseConverter:: vslam_pose_callback, this, std::placeholders::_1));

    // /odom 토픽에서 Odometry 메시지를 구독합니다.
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&PoseConverter::odom_callback, this, std::placeholders::_1)
    );
    

    tracking_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/run_slam/tracking_status", 10,
      std::bind(&PoseConverter::tracking_callback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
      50ms,
      std::bind(&PoseConverter::publish_transform, this));

    latest_pose_available_ = false;
    prev_tracking_ = false;
    current_tracking = false;
    scale_factor = 1.0;

    reset_odom_client_ = this->create_client<omo_r1_interfaces::srv::ResetOdom>("reset_odom");
    set_tf_only_mode_client_ = this->create_client<omo_r1_interfaces::srv::Onoff>("set_tf_only_mode");

  }

private:
  void  vslam_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_pose_ = *msg;
    latest_pose_available_ = true;
  }

  void publish_transform() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";
    
    if (latest_pose_available_) {
      curr_map_position.x = latest_pose_.pose.pose.position.x * 10.0;
      curr_map_position.y = latest_pose_.pose.pose.position.y * 10.0;
      t.transform.translation.x = curr_map_position.x;
      t.transform.translation.y = curr_map_position.y;
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
      // 0.5초 전과 현재의 이동량(변위) 계산 (평면상의 Euclidean 거리)
      double delta_map = std::sqrt(std::pow(curr_map_position.x - prev_map_position.x, 2) + std::pow(curr_map_position.y - prev_map_position.y, 2));
      double delta_odom_base = std::sqrt(std::pow(curr_odom_position.x - prev_odom_position.x, 2) + std::pow(curr_odom_position.y - prev_odom_position.y, 2));

      // 두 변위의 비율 계산 (분모가 0이면 1.0으로 설정)
      scale_factor = (delta_odom_base > 1e-6) ? (delta_map / delta_odom_base) : 1.0;
      RCLCPP_INFO(this->get_logger(), "Calculated scale_factor (delta ratio): %.2f", scale_factor);

      call_reset_odom();
      //리셋 대신 map->odom 변환에서 좌표 수정
    }

    prev_odom_position = curr_odom_position;
    prev_map_position = curr_map_position;
    tf_broadcaster_->sendTransform(t);
  }

  void tracking_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    current_tracking = msg->data;
  }

  void call_reset_odom() {
    // if (!reset_odom_client_->wait_for_service(std::chrono::seconds(1))) {
    //   RCLCPP_ERROR(this->get_logger(), "reset_odom service not available");
    //   return;
    // }
    auto request = std::make_shared<omo_r1_interfaces::srv::ResetOdom::Request>();
    request->x = 0.0;
    request->y = 0.0;
    request->theta = 0.0;
    //RCLCPP_INFO(this->get_logger(), "Calling reset_odom service with x=0, y=0, theta=0");
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

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    curr_odom_position = msg->pose.pose.position;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vslam_pose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tracking_subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr param_timer_;
  nav_msgs::msg::Odometry latest_pose_;
  geometry_msgs::msg::Point curr_odom_position;
  geometry_msgs::msg::Point prev_odom_position;
  geometry_msgs::msg::Point curr_map_position;
  geometry_msgs::msg::Point prev_map_position;
  bool latest_pose_available_;
  bool prev_tracking_;
  bool current_tracking;
  double scale_factor;
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
