#include "human_detector/detector_logic.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>

namespace human_detector {

DetectorLogic::DetectorLogic() {}

std::vector<Location> DetectorLogic::scanToWorldCoordinates(
    const sensor_msgs::msg::LaserScan::SharedPtr scan,
    const geometry_msgs::msg::PoseStamped& robot_pose
) {
    std::vector<Location> world_points;
    
    // Extract robot yaw from quaternion
    tf2::Quaternion q(
        robot_pose.pose.orientation.x,
        robot_pose.pose.orientation.y,
        robot_pose.pose.orientation.z,
        robot_pose.pose.orientation.w
    );
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    // Convert laser scan to world coordinates
    float angle = scan->angle_min;
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        float range = scan->ranges[i];
        
        if (range >= scan->range_min && range <= scan->range_max * 0.95) {
            // Point in robot frame
            double local_x = range * cos(angle);
            double local_y = range * sin(angle);
            
            // Transform to world frame
            Location world_point;
            world_point.x = robot_pose.pose.position.x + 
                           local_x * cos(yaw) - local_y * sin(yaw);
            world_point.y = robot_pose.pose.position.y + 
                           local_x * sin(yaw) + local_y * cos(yaw);
            
            world_points.push_back(world_point);
        }
        
        angle += scan->angle_increment;
    }
    
    return world_points;
}

std::vector<HumanCluster> DetectorLogic::clusterAnomalies(const std::vector<Location>& anomalies) {
    if (anomalies.empty()) {
        return {};
    }
    
    std::vector<HumanCluster> clusters;
    std::vector<bool> assigned(anomalies.size(), false);
    
    for (size_t i = 0; i < anomalies.size(); ++i) {
        if (assigned[i]) continue;
        
        HumanCluster cluster;
        cluster.points.push_back(anomalies[i]);
        assigned[i] = true;
        
        // Find all nearby points
        for (size_t j = i + 1; j < anomalies.size(); ++j) {
            if (assigned[j]) continue;
            
            // Check distance to any point in current cluster
            bool is_close = false;
            for (const auto& cluster_point : cluster.points) {
                if (cluster_point.distance(anomalies[j]) < cluster_distance_) {
                    is_close = true;
                    break;
                }
            }
            
            if (is_close) {
                cluster.points.push_back(anomalies[j]);
                assigned[j] = true;
            }
        }
        
        // Only keep clusters with enough points
        if (cluster.points.size() >= static_cast<size_t>(min_cluster_size_)) {
            // Calculate cluster center
            double sum_x = 0.0, sum_y = 0.0;
            for (const auto& point : cluster.points) {
                sum_x += point.x;
                sum_y += point.y;
            }
            cluster.center.x = sum_x / cluster.points.size();
            cluster.center.y = sum_y / cluster.points.size();
            cluster.point_count = cluster.points.size();
            
            clusters.push_back(cluster);
        }
    }
    
    return clusters;
}

std::vector<HumanCluster> DetectorLogic::detectHumans(
    const sensor_msgs::msg::LaserScan::SharedPtr scan,
    const nav_msgs::msg::OccupancyGrid::SharedPtr map,
    const geometry_msgs::msg::PoseStamped& robot_pose
) {
    auto world_points = scanToWorldCoordinates(scan, robot_pose);
    std::vector<Location> anomalies;
    
    // Check each point against the map
    for (const auto& point : world_points) {
        // Convert world coordinates to map grid coordinates
        int grid_x = static_cast<int>((point.x - map->info.origin.position.x) / 
                                      map->info.resolution);
        int grid_y = static_cast<int>((point.y - map->info.origin.position.y) / 
                                      map->info.resolution);
        
        // Check if within map bounds
        if (grid_x >= 0 && grid_x < static_cast<int>(map->info.width) &&
            grid_y >= 0 && grid_y < static_cast<int>(map->info.height)) {
            
            int index = grid_y * map->info.width + grid_x;
            
            // If map says free (value close to 0) but we detect obstacle, it's an anomaly
            if (map->data[index] >= 0 && map->data[index] < map_threshold_) {
                anomalies.push_back(point);
            }
        }
    }
    
    // Cluster anomalies into distinct objects (potential humans)
    return clusterAnomalies(anomalies);
}

} // namespace human_detector
