#ifndef DETECTOR_LOGIC_HPP
#define DETECTOR_LOGIC_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <cmath>

namespace human_detector {

struct Location {
    double x;
    double y;
    
    double distance(const Location& other) const {
        return std::hypot(x - other.x, y - other.y);
    }
};

struct HumanCluster {
    Location center;
    int point_count;
    std::vector<Location> points;
};

class DetectorLogic {
public:
    DetectorLogic();
    
    // Detect and cluster anomalies into potential human locations
    std::vector<HumanCluster> detectHumans(
        const sensor_msgs::msg::LaserScan::SharedPtr scan,
        const nav_msgs::msg::OccupancyGrid::SharedPtr map,
        const geometry_msgs::msg::PoseStamped& robot_pose
    );
    
    // Convert laser scan points to world coordinates
    std::vector<Location> scanToWorldCoordinates(
        const sensor_msgs::msg::LaserScan::SharedPtr scan,
        const geometry_msgs::msg::PoseStamped& robot_pose
    );
    
    // Cluster nearby anomalies into objects
    std::vector<HumanCluster> clusterAnomalies(const std::vector<Location>& anomalies);

private:
    double cluster_distance_ = 0.8;  // meters - points within this distance belong to same cluster
    int min_cluster_size_ = 5;        // minimum points to be considered a human
    double map_threshold_ = 50;       // occupancy threshold for "occupied" in map
};

} // namespace human_detector

#endif // DETECTOR_LOGIC_HPP
