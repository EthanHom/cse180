// CLAUDE SONNET 4.5 THINKING
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <cmath>

class ChaserNode : public rclcpp::Node
{
public:
    ChaserNode() : Node("chaser")
    {
        // Subscribe to both turtle poses
        turtle1_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&ChaserNode::turtle1PoseCallback, this, std::placeholders::_1));
        
        t1_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/T1/pose", 10,
            std::bind(&ChaserNode::t1PoseCallback, this, std::placeholders::_1));
        
        // Publisher to move T1
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/T1/cmd_vel", 10);
        
        // Timer to publish commands at 10Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ChaserNode::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Chaser node started");
    }

private:
    void turtle1PoseCallback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        turtle1_x_ = msg->x;
        turtle1_y_ = msg->y;
        turtle1_received_ = true;
    }
    
    void t1PoseCallback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        t1_x_ = msg->x;
        t1_y_ = msg->y;
        t1_theta_ = msg->theta;
        t1_received_ = true;
    }
    
    void controlLoop()
    {
        if (!turtle1_received_ || !t1_received_) {
            return;
        }
        
        // Calculate distance and angle to turtle1
        double dx = turtle1_x_ - t1_x_;
        double dy = turtle1_y_ - t1_y_;
        double distance = std::sqrt(dx * dx + dy * dy);
        double angle_to_target = std::atan2(dy, dx);
        
        // Calculate angular error
        double angle_error = angle_to_target - t1_theta_;
        
        // Normalize angle to [-pi, pi]
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;
        
        // Create velocity command
        auto cmd = geometry_msgs::msg::Twist();
        
        if (distance > 0.5) {  // If far from target
            cmd.linear.x = std::min(2.0 * distance, 2.0);  // Proportional control, max speed 2.0
            cmd.angular.z = 4.0 * angle_error;  // Turn towards target
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        }
        
        cmd_vel_pub_->publish(cmd);
    }
    
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr t1_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double turtle1_x_ = 0.0, turtle1_y_ = 0.0;
    double t1_x_ = 0.0, t1_y_ = 0.0, t1_theta_ = 0.0;
    bool turtle1_received_ = false;
    bool t1_received_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChaserNode>());
    rclcpp::shutdown();
    return 0;
}
// CLAUDE SONNET 4.5 THINKING END