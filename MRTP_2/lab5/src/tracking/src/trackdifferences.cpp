#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class TrackDifferences : public rclcpp::Node
{
public:
  TrackDifferences()
  : Node("trackdifferences")
  {
    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribe to amcl_pose (pose in map frame)
    amcl_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10, std::bind(&TrackDifferences::amcl_callback, this, std::placeholders::_1));

    // Subscribe to odom (pose in odom frame)
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&TrackDifferences::odom_callback, this, std::placeholders::_1));

    // Create timer to compute and print differences
    timer_ = this->create_wall_timer(
      500ms, std::bind(&TrackDifferences::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "TrackDifferences node has been started.");
  }

private:
  void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    amcl_pose_ = msg->pose.pose;
    has_amcl_ = true;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_pose_ = msg->pose.pose;
    has_odom_ = true;
  }

  double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& quat)
  {
    tf2::Quaternion tf_quat;
    tf2::fromMsg(quat, tf_quat);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    
    return yaw;
  }

  void timer_callback()
  {
    if (!has_amcl_ || !has_odom_) {
      RCLCPP_INFO(this->get_logger(), "Waiting for pose data...");
      return;
    }

    try {
      // Get transform from odom to map
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_->lookupTransform(
        "map", "odom", tf2::TimePointZero);

      // Transform odom pose to map frame
      geometry_msgs::msg::PoseStamped odom_pose_stamped;
      odom_pose_stamped.header.frame_id = "odom";
      odom_pose_stamped.pose = odom_pose_;

      geometry_msgs::msg::PoseStamped odom_in_map;
      tf2::doTransform(odom_pose_stamped, odom_in_map, transform_stamped);

      // Calculate differences
      double diff_x = amcl_pose_.position.x - odom_in_map.pose.position.x;
      double diff_y = amcl_pose_.position.y - odom_in_map.pose.position.y;
      
      double yaw_amcl = quaternion_to_yaw(amcl_pose_.orientation);
      double yaw_odom = quaternion_to_yaw(odom_in_map.pose.orientation);
      double diff_yaw = yaw_amcl - yaw_odom;

      // Normalize yaw difference to [-pi, pi]
      while (diff_yaw > M_PI) diff_yaw -= 2.0 * M_PI;
      while (diff_yaw < -M_PI) diff_yaw += 2.0 * M_PI;

      // Print differences
      RCLCPP_INFO(this->get_logger(), 
        "Differences -> X: %.4f m, Y: %.4f m, Yaw: %.4f rad (%.2f deg)",
        diff_x, diff_y, diff_yaw, diff_yaw * 180.0 / M_PI);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), 
        "Could not transform odom to map: %s", ex.what());
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  
  geometry_msgs::msg::Pose amcl_pose_;
  geometry_msgs::msg::Pose odom_pose_;
  bool has_amcl_ = false;
  bool has_odom_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackDifferences>());
  rclcpp::shutdown();
  return 0;
}
