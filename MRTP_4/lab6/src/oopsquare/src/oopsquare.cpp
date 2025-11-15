#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class OopSquare : public rclcpp::Node
{
public:
    OopSquare() : Node("oopsquare")
    {
        // Publisher for velocity commands
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1000);
        
        // Subscriber for odometry
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 1000,
            std::bind(&OopSquare::odomCallback, this, std::placeholders::_1));
        
        // Initialize member variables (no globals per instructions)
        init_ = false;
        rotate_ = false;
        start_x_ = 0.0;
        start_y_ = 0.0;
        direction_ = 0;
        
        // Desired directions (matching Listing 4.2)
        DIRS_[0] = 0.0;
        DIRS_[1] = M_PI / 2.0;
        DIRS_[2] = M_PI;
        DIRS_[3] = -M_PI / 2.0;
        
        // Timer for control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&OopSquare::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "OopSquare node initialized");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract position
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        
        // Extract yaw from quaternion
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, theta_);
        
        // Initialize on first reading
        if (!init_) {
            start_x_ = x_;
            start_y_ = y_;
            init_ = true;
        }
    }
    
    void controlLoop()
    {
        if (!init_) {
            // Wait for first odometry reading
            return;
        }
        
        geometry_msgs::msg::Twist msg;
        
        if (rotate_) {
            // Rotating phase - logic from Listing 4.2
            if (((direction_ == 0) && (theta_ < DIRS_[0])) ||
                ((direction_ == 1) && (theta_ < DIRS_[1])) ||
                ((direction_ == 2) && (theta_ > 0)) ||
                ((direction_ == 3) && (theta_ < DIRS_[3]))) {
                // Keep rotating
                msg.linear.x = 0.0;
                msg.angular.z = M_PI / 8.0;
            } else {
                // Reached desired heading
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
                pub_->publish(msg);
                
                rotate_ = false;
                start_x_ = x_;
                start_y_ = y_;
                
                RCLCPP_INFO(this->get_logger(), "Aligned to direction %d", direction_);
            }
        } else {
            // Translating phase - logic from Listing 4.2
            if (std::hypot(x_ - start_x_, y_ - start_y_) < 1.0) {
                // Keep moving forward (1 meter side)
                msg.linear.x = 0.5;
                msg.angular.z = 0.0;
            } else {
                // Moved 1 meter
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
                pub_->publish(msg);
                
                rotate_ = true;
                direction_ = (direction_ + 1) % 4;
                
                RCLCPP_INFO(this->get_logger(), "Completed side %d", direction_);
                
                // After 4 sides, stop
                if (direction_ == 0) {
                    RCLCPP_INFO(this->get_logger(), "Square completed!");
                    rclcpp::shutdown();
                    return;
                }
            }
        }
        
        pub_->publish(msg);
    }
    
    // Member variables (no globals per instructions)
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    bool init_;
    bool rotate_;
    double x_, y_, theta_;
    double start_x_, start_y_;
    int direction_;
    double DIRS_[4];
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OopSquare>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
