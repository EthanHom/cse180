// CLAUDE SONNET 4.0 THINKING

// #include <chrono>
// #include <memory>
// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"

// using namespace std::chrono_literals;

// class ControlTurtlebot : public rclcpp::Node
// {
// public:
//     ControlTurtlebot() : Node("control_turtlebot"), moving_straight_(true)
//     {
//         publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
//         timer_ = this->create_wall_timer(1000ms, std::bind(&ControlTurtlebot::timer_callback, this));
//         RCLCPP_INFO(this->get_logger(), "Control Turtlebot node started");
//     }

// private:
//     void timer_callback()
//     {
//         auto message = geometry_msgs::msg::Twist();
        
//         if (moving_straight_) {
//             message.linear.x = 1.5;  // Move forward at 1.5 m/s
//             message.angular.z = 0.0;
//             RCLCPP_INFO(this->get_logger(), "Moving straight at 1.5 m/s");
//         } else {
//             message.linear.x = 0.0;
//             message.angular.z = -0.6;  // Rotate clockwise at 0.6 rad/s
//             RCLCPP_INFO(this->get_logger(), "Rotating clockwise at 0.6 rad/s");
//         }
        
//         publisher_->publish(message);
//         moving_straight_ = !moving_straight_;
//     }
    
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     bool moving_straight_;
// };

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ControlTurtlebot>());
//     rclcpp::shutdown();
//     return 0;
// }



#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ControlTurtlebot : public rclcpp::Node
{
public:
    ControlTurtlebot() : Node("control_turtlebot")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Control Turtlebot node started");
    }
    
    void run()
    {
        // Create Rate instance for 1 Hz frequency
        rclcpp::Rate rate(1.0);  // 1 Hz = 1 time per second
        bool moving_straight = true;
        
        while (rclcpp::ok()) {
            auto message = geometry_msgs::msg::Twist();
            
            if (moving_straight) {
                message.linear.x = 1.5;  // Forward at 1.5 m/s
                message.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Moving straight at 1.5 m/s");
            } else {
                message.linear.x = 0.0;
                message.angular.z = -0.6;  // Clockwise at 0.6 rad/s
                RCLCPP_INFO(this->get_logger(), "Rotating clockwise at 0.6 rad/s");
            }
            
            publisher_->publish(message);
            moving_straight = !moving_straight;  // Alternate states
            
            rate.sleep();  // Sleep to maintain 1 Hz frequency
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlTurtlebot>();
    node->run();  // Call the run method instead of spin
    rclcpp::shutdown();
    return 0;
}
