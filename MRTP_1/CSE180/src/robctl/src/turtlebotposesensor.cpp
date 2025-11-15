// CLAUDE SONNET 4.0 THINKING

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

class TurtlebotPoseSensor : public rclcpp::Node
{
public:
    TurtlebotPoseSensor() : Node("turtlebot_pose_sensor")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TurtlebotPoseSensor::odom_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Turtlebot Pose Sensor node started");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;
        
        RCLCPP_INFO(this->get_logger(), "Turtlebot position: (%.3f, %.3f, %.3f)", x, y, z);
    }
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlebotPoseSensor>());
    rclcpp::shutdown();
    return 0;
}
