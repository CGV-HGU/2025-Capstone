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
    prev_map_position_.x = prev_map_position_.y = prev_map_position_.z = 0.0;
    prev_odom_position_.x = prev_odom_position_.y = prev_odom_position_.z = 0.0;
    latest_pose_available_ = false;
    first_pose_received_ = false;
    prev_tracking_ = false;
    current_tracking_ = false;
    // scale_factor_ = 10.0; //Fixed to 10.0

    scale_factor_ = 2.5; // when the resolution is 0.05, scale_factor = 100 * 0.05 


    vslam_pose_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/run_slam/camera_pose", 10,
      std::bind(&PoseConverter::vslam_pose_callback, this, std::placeholders::_1));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&PoseConverter::odom_callback, this, std::placeholders::_1));

    tracking_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/run_slam/tracking_status", 10,
      std::bind(&PoseConverter::tracking_callback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
      50ms,
      std::bind(&PoseConverter::publish_transform, this));

    reset_odom_client_ = this->create_client<omo_r1_interfaces::srv::ResetOdom>("reset_odom");
    set_tf_only_mode_client_ = this->create_client<omo_r1_interfaces::srv::Onoff>("set_tf_only_mode");
  }

private:
  void vslam_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!first_pose_received_) {
      curr_pose_ = *msg;
      curr_map_position_.x = curr_pose_.pose.pose.position.x * scale_factor_;
      curr_map_position_.y = curr_pose_.pose.pose.position.y * scale_factor_;
      prev_map_position_ = curr_map_position_;
      prev_odom_position_ = curr_odom_position_;
      first_pose_received_ = true;
      latest_pose_available_ = true;
      return;
    }

    if (current_tracking_) {
      latest_pose_ = *msg;
      latest_pose_available_ = true;
      double delta_map = std::hypot(
        latest_pose_.pose.pose.position.x * scale_factor_ - prev_map_position_.x,
        latest_pose_.pose.pose.position.y * scale_factor_ - prev_map_position_.y);
      if (delta_map < 1.0) {
        curr_pose_ = *msg;
      }
    } else {
      curr_pose_ = *msg;
    }
  }

  void publish_transform() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";
    
    if (latest_pose_available_) {
      curr_map_position_.x = curr_pose_.pose.pose.position.x * scale_factor_;
      curr_map_position_.y = curr_pose_.pose.pose.position.y * scale_factor_;
      t.transform.translation.x = curr_map_position_.x;
      t.transform.translation.y = curr_map_position_.y;
      t.transform.translation.z = 0.0;
      t.transform.rotation.x = curr_pose_.pose.pose.orientation.x;
      t.transform.rotation.y = curr_pose_.pose.pose.orientation.y * -1;
      t.transform.rotation.z = curr_pose_.pose.pose.orientation.z;
      t.transform.rotation.w = curr_pose_.pose.pose.orientation.w;
    } else {
      t.transform.translation.x = 0.0;
      t.transform.translation.y = 0.0;
      t.transform.translation.z = 0.0;
      t.transform.rotation.x = 0.0;
      t.transform.rotation.y = 0.0;
      t.transform.rotation.z = 0.0;
      t.transform.rotation.w = 1.0;
      call_reset_odom();
    }
    
    if(current_tracking_) {
      call_reset_odom();
    }

    prev_odom_position_ = curr_odom_position_;
    prev_map_position_ = curr_map_position_;
    tf_broadcaster_->sendTransform(t);
  }

  void tracking_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    prev_tracking_ = current_tracking_;
    current_tracking_ = msg->data;
  }

  void call_reset_odom() {
    // if (!reset_odom_client_->wait_for_service(1s)) {
    //   RCLCPP_ERROR(this->get_logger(), "reset_odom service not available");
    //   return;
    // }
    auto request = std::make_shared<omo_r1_interfaces::srv::ResetOdom::Request>();
    request->x = 0.0;
    request->y = 0.0;
    request->theta = 0.0;
    reset_odom_client_->async_send_request(request);
  }

  void call_set_tf_only_mode(bool enable) {
    if (!set_tf_only_mode_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "set_tf_only_mode service not available");
      return;
    }
    auto request = std::make_shared<omo_r1_interfaces::srv::Onoff::Request>();
    request->set = enable;
    RCLCPP_INFO(this->get_logger(), "Calling set_tf_only_mode: %s", enable ? "true" : "false");
    set_tf_only_mode_client_->async_send_request(request);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    curr_odom_position_ = msg->pose.pose.position;
  }

  // Subscriptions, timers, broadcasters, clients
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vslam_pose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tracking_subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  nav_msgs::msg::Odometry latest_pose_;
  nav_msgs::msg::Odometry curr_pose_;
  geometry_msgs::msg::Point curr_odom_position_;
  geometry_msgs::msg::Point prev_odom_position_;
  geometry_msgs::msg::Point curr_map_position_;
  geometry_msgs::msg::Point prev_map_position_;
  bool latest_pose_available_;
  bool first_pose_received_;
  bool prev_tracking_;
  bool current_tracking_;
  double scale_factor_;

  // Service clients
  rclcpp::Client<omo_r1_interfaces::srv::ResetOdom>::SharedPtr reset_odom_client_;
  rclcpp::Client<omo_r1_interfaces::srv::Onoff>::SharedPtr set_tf_only_mode_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseConverter>());
  rclcpp::shutdown();
  return 0;
}
