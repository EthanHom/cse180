#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "human_detector/detector_logic.hpp"
#include <set>

class HumanDetectionNode : public rclcpp::Node {
public:
    HumanDetectionNode() : Node("human_detection_node") {
        this->declare_parameter<std::string>("scan_topic", "/scan");
        this->declare_parameter<std::string>("map_topic", "/map");
        this->declare_parameter<std::string>("pose_topic", "/amcl_pose");
        
        std::string scan_topic = this->get_parameter("scan_topic").as_string();
        std::string map_topic = this->get_parameter("map_topic").as_string();
        std::string pose_topic = this->get_parameter("pose_topic").as_string();
        
        RCLCPP_INFO(this->get_logger(), "Subscribing to scan topic: %s", scan_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribing to map topic: %s", map_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribing to pose topic: %s", pose_topic.c_str());
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, 10,
            std::bind(&HumanDetectionNode::scanCallback, this, std::placeholders::_1)
        );
        
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic, rclcpp::QoS(10).transient_local(),
            std::bind(&HumanDetectionNode::mapCallback, this, std::placeholders::_1)
        );
        
        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic, 10,
            std::bind(&HumanDetectionNode::amclCallback, this, std::placeholders::_1)
        );
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        detector_ = std::make_unique<human_detector::DetectorLogic>();
        
        RCLCPP_INFO(this->get_logger(), "Human Detection Node initialized");
        
        timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&HumanDetectionNode::checkEnvironment, this)
        );
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
        if (!scan_received_) {
            RCLCPP_INFO(this->get_logger(), "First scan received! %zu points", msg->ranges.size());
            scan_received_ = true;
        }
    }
    
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_ = msg;
        if (!map_received_) {
            RCLCPP_INFO(this->get_logger(), "Map received! Dimensions: %dx%d",
                        msg->info.width, msg->info.height);
            map_received_ = true;
        }
    }
    
    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        current_pose_.header = msg->header;
        current_pose_.pose = msg->pose.pose;
        if (!pose_received_) {
            RCLCPP_INFO(this->get_logger(), "Pose received! Position: (%.2f, %.2f)",
                        msg->pose.pose.position.x, msg->pose.pose.position.y);
            pose_received_ = true;
        }
    }
    
    void checkEnvironment() {
        if (!latest_scan_ || !map_) {
            RCLCPP_WARN(this->get_logger(), 
                "Waiting for data... [scan: %s, map: %s, pose: %s]",
                latest_scan_ ? "✓" : "✗",
                map_ ? "✓" : "✗",
                pose_received_ ? "✓" : "✗");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "=== Checking for environment changes ===");
        
        auto human_clusters = detector_->detectHumans(latest_scan_, map_, current_pose_);
        
        if (!human_clusters.empty()) {
            RCLCPP_INFO(this->get_logger(), "Detected %zu potential human(s)!", human_clusters.size());
            
            for (size_t i = 0; i < human_clusters.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), 
                    "  Human %zu: Location (%.2f, %.2f) with %d laser points",
                    i + 1, 
                    human_clusters[i].center.x, 
                    human_clusters[i].center.y,
                    human_clusters[i].point_count);
            }
            
            // Track unique human locations over time
            for (const auto& cluster : human_clusters) {
                bool is_new = true;
                for (const auto& known : detected_humans_) {
                    if (known.distance(cluster.center) < 1.0) {  // Within 1 meter = same human
                        is_new = false;
                        break;
                    }
                }
                if (is_new) {
                    detected_humans_.push_back(cluster.center);
                    RCLCPP_WARN(this->get_logger(), 
                        "*** NEW HUMAN DETECTED at (%.2f, %.2f) ***",
                        cluster.center.x, cluster.center.y);
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "Total unique humans tracked: %zu", detected_humans_.size());
        } else {
            RCLCPP_INFO(this->get_logger(), "No humans/anomalies detected in current view.");
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    geometry_msgs::msg::PoseStamped current_pose_;
    
    bool scan_received_ = false;
    bool map_received_ = false;
    bool pose_received_ = false;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<human_detector::DetectorLogic> detector_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<human_detector::Location> detected_humans_;  // Track unique human locations
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HumanDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
