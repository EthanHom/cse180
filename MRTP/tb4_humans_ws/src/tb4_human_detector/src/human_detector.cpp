// run the project in terminal: 
/* 

Terminal 1:
make vnc
make bash
cd /MRTP/MRTP/
source install/setup.bash
ros2 launch gazeboenvs tb4_warehouse.launch.py use_rviz:=true

Terminal 2:
make shell
cd /MRTP/tb4_humans_ws/
colcon build
source install/setup.bash
ros2 run tb4_human_detector human_detector_node 

*/

#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <map>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "navigation/navigation.hpp"

struct Cluster {
    double sum_x{0.0};
    double sum_y{0.0};
    int count{0};
    double cx() const { return (count > 0) ? sum_x / count : 0.0; }
    double cy() const { return (count > 0) ? sum_y / count : 0.0; }
};

struct LocationScan {
    double x, y;
    int total_scans{0};
    int hit_scans{0};
    double hit_ratio() const { return total_scans > 0 ? (double)hit_scans / total_scans : 0.0; }
};

class HumanDetector : public rclcpp::Node {
public:
    HumanDetector()
        : Node("tb4_human_detector"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        if (!this->has_parameter("use_sim_time")) {
            this->declare_parameter("use_sim_time", true);
        }
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        // Initialize known static obstacles
        initializeKnownObstacles();

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", rclcpp::QoS(10).transient_local().reliable(),
            std::bind(&HumanDetector::mapCallback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            std::bind(&HumanDetector::scanCallback, this, std::placeholders::_1));

        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", 10,
            std::bind(&HumanDetector::amclCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "HumanDetector initialized with %zu known obstacles.", 
            known_obstacles_.size());
    }

    bool hasMap() const { return have_map_.load(); }
    
    geometry_msgs::msg::Pose getCurrentPose() {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        return current_pose_;
    }

    void resetDetection() {
        std::lock_guard<std::mutex> lock(cluster_mutex_);
        clusters_.clear();
        human1_close_range_hits_.clear();
        human2_close_range_hits_.clear();
        dynamic_detections_.clear();
        scan_count_.store(0);
        scans_near_h1_.store(0);
        scans_near_h2_.store(0);
        RCLCPP_INFO(this->get_logger(), "Detection counters reset.");
    }

    void printDebugInfo() {
        std::lock_guard<std::mutex> lock(cluster_mutex_);
        RCLCPP_INFO(this->get_logger(), 
            "[DEBUG] Scans: %d, Near H1: %d (hits: %zu), Near H2: %d (hits: %zu), DynDetections: %zu",
            scan_count_.load(), scans_near_h1_.load(), human1_close_range_hits_.size(),
            scans_near_h2_.load(), human2_close_range_hits_.size(), dynamic_detections_.size());
    }

    void reportHumans() {
        std::lock_guard<std::mutex> lock(cluster_mutex_);
        
        int scans_near_1 = scans_near_h1_.load();
        int scans_near_2 = scans_near_h2_.load();
        
        int h1_hit_scans = human1_close_range_hits_.size();
        int h2_hit_scans = human2_close_range_hits_.size();
        
        RCLCPP_INFO(this->get_logger(), "==========================================");
        RCLCPP_INFO(this->get_logger(), "        HUMAN DETECTION REPORT            ");
        RCLCPP_INFO(this->get_logger(), "==========================================");
        RCLCPP_INFO(this->get_logger(), "Scans near H1: %d (hits: %d)", scans_near_1, h1_hit_scans);
        RCLCPP_INFO(this->get_logger(), "Scans near H2: %d (hits: %d)", scans_near_2, h2_hit_scans);
        
        bool h1_present = false;
        bool h2_present = false;
        
        if (scans_near_1 > 50) {
            double h1_hit_ratio = (double)h1_hit_scans / scans_near_1;
            RCLCPP_INFO(this->get_logger(), "H1 hit ratio: %.2f", h1_hit_ratio);
            h1_present = (h1_hit_ratio > 0.3);
        }
        
        if (scans_near_2 > 50) {
            double h2_hit_ratio = (double)h2_hit_scans / scans_near_2;
            RCLCPP_INFO(this->get_logger(), "H2 hit ratio: %.2f", h2_hit_ratio);
            h2_present = (h2_hit_ratio > 0.3);
        }

        if (h1_present) {
            RCLCPP_INFO(this->get_logger(), "[Human 1] STILL at (%.2f, %.2f) ✓", human1_x_, human1_y_);
        } else {
            RCLCPP_INFO(this->get_logger(), "[Human 1] MOVED from (%.2f, %.2f)", human1_x_, human1_y_);
        }

        if (h2_present) {
            RCLCPP_INFO(this->get_logger(), "[Human 2] STILL at (%.2f, %.2f) ✓", human2_x_, human2_y_);
        } else {
            RCLCPP_INFO(this->get_logger(), "[Human 2] MOVED from (%.2f, %.2f)", human2_x_, human2_y_);
        }

        if (h1_present && h2_present) {
            RCLCPP_INFO(this->get_logger(), "Both humans at original locations.");
            return;
        }

        // ================================================================
        // IMPROVED: Filter out known static obstacles
        // ================================================================
        RCLCPP_INFO(this->get_logger(), "------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Analyzing detections (filtering static obstacles)...");

        std::vector<std::pair<double, double>> candidate_locations;
        
        for (const auto& [key, detection] : dynamic_detections_) {
            if (detection.total_scans < 10) continue;
            if (detection.hit_scans < 3) continue;
            
            double ratio = detection.hit_ratio();
            if (ratio < 0.05) continue;  // At least 5% hit rate
            
            // FILTER 1: Away from original human positions
            double d1 = std::hypot(detection.x - human1_x_, detection.y - human1_y_);
            double d2 = std::hypot(detection.x - human2_x_, detection.y - human2_y_);
            if (d1 < 2.0 || d2 < 2.0) continue;
            
            // FILTER 2: NOT a known static obstacle!
            if (isKnownStaticObstacle(detection.x, detection.y)) {
                continue;  // Skip shelves, tables, barriers, chairs
            }
            
            // FILTER 3: Within reasonable bounds for human relocation
            if (detection.x < -13.0 || detection.x > 13.0) continue;  // Outside expected area
            if (detection.y < -20.0 || detection.y > 23.0) continue;
            
            candidate_locations.push_back({detection.x, detection.y});
            RCLCPP_INFO(this->get_logger(), 
                "  Valid candidate: (%.2f, %.2f) - %d/%d scans (%.1f%%)",
                detection.x, detection.y, 
                detection.hit_scans, detection.total_scans, ratio * 100);
        }

        // Cluster nearby candidates (merge close detections)
        std::vector<std::pair<double, double>> clustered_locations;
        for (const auto& loc : candidate_locations) {
            bool merged = false;
            for (auto& existing : clustered_locations) {
                if (std::hypot(loc.first - existing.first, loc.second - existing.second) < 1.5) {
                    existing.first = (existing.first + loc.first) / 2.0;
                    existing.second = (existing.second + loc.second) / 2.0;
                    merged = true;
                    break;
                }
            }
            if (!merged) {
                clustered_locations.push_back(loc);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Found %zu potential human locations (after filtering)", 
            clustered_locations.size());

        // Report new locations
        if (!h1_present) {
            if (!clustered_locations.empty()) {
                RCLCPP_INFO(this->get_logger(), 
                    "=> Human 1 NEW LOCATION: (%.2f, %.2f) ✓",
                    clustered_locations[0].first, clustered_locations[0].second);
            } else {
                RCLCPP_WARN(this->get_logger(), "=> Human 1 new location: NOT FOUND");
            }
        }

        if (!h2_present) {
            size_t idx = h1_present ? 0 : 1;
            if (idx < clustered_locations.size()) {
                RCLCPP_INFO(this->get_logger(), 
                    "=> Human 2 NEW LOCATION: (%.2f, %.2f) ✓",
                    clustered_locations[idx].first, clustered_locations[idx].second);
            } else {
                RCLCPP_WARN(this->get_logger(), "=> Human 2 new location: NOT FOUND");
            }
        }
    }

private:
    std::mutex map_mutex_;
    std::mutex pose_mutex_;
    std::mutex cluster_mutex_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

    nav_msgs::msg::OccupancyGrid map_;
    std::atomic<bool> have_map_{false};
    geometry_msgs::msg::Pose current_pose_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::vector<Cluster> clusters_;
    std::map<std::pair<int,int>, LocationScan> dynamic_detections_;
    
    // Known static obstacles
    std::vector<std::pair<double, double>> known_obstacles_;

    const double cluster_dist_thresh_ = 0.5;

    const double human1_x_ = 1.00;
    const double human1_y_ = -1.00;
    const double human2_x_ = -12.00;
    const double human2_y_ = 15.00;
    
    const double detection_radius_ = 0.8;
    const double near_threshold_ = 2.5;
    
    std::map<int, bool> human1_close_range_hits_;
    std::map<int, bool> human2_close_range_hits_;
    
    std::atomic<int> scan_count_{0};
    std::atomic<int> scans_near_h1_{0};
    std::atomic<int> scans_near_h2_{0};

    void initializeKnownObstacles() {
        // All known static obstacles from the warehouse
        known_obstacles_ = {
            {-8.5, -13.0},   // SHELF_BIG_0
            {6.5, -13.0},    // SHELF_BIG_1
            {-1.5, -13.0},   // SHELF_BIG_2
            {13.5, 4.5},     // SHELF_3
            {10.0, 4.5},     // SHELF_4
            {13.5, -21.0},   // SHELF_5
            {13.5, -15.0},   // SHELF_6
            {0.4, -2.0},     // SHELF_7
            {3.5, 9.5},      // SHELF_BIG_3
            {-1.3, 18.5},    // SHELF_BIG_4
            {-10.0, 21.5},   // SHELF_0
            {-7.0, 23.6},    // SHELF_1
            {-4.0, 21.5},    // SHELF_2
            {-10.4, 14.75},  // BARRIER_0
            {-10.4, 10.5},   // BARRIER_1
            {-10.4, 6.5},    // BARRIER_2
            {-12.85, 4.85},  // BARRIER_3
            {14.3, -5.5},    // CHAIR_0
            {14.3, -4.0},    // CHAIR_1
            {-11.5, 6.4},    // FCHAIR_0
            {-14.0, 6.5},    // FCHAIR_1
            {-12.7, 6.5},    // TABLE_0
        };
    }

    bool isKnownStaticObstacle(double x, double y) const {
        const double threshold = 2.0;  // Within 2m of a known obstacle
        for (const auto& obstacle : known_obstacles_) {
            double dist = std::hypot(x - obstacle.first, y - obstacle.second);
            if (dist < threshold) {
                return true;  // This is a static obstacle
            }
        }
        return false;  // Not near any known static obstacle - likely a human!
    }

    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_ = msg->pose.pose;
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_ = *msg;
        have_map_.store(true);
        RCLCPP_INFO(this->get_logger(), "Map received: %ux%u", map_.info.width, map_.info.height);
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        geometry_msgs::msg::TransformStamped tf;
        
        try {
            tf = tf_buffer_.lookupTransform("map", scan->header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            return;
        }

        if (!have_map_.load()) return;

        double robot_x = tf.transform.translation.x;
        double robot_y = tf.transform.translation.y;
        
        int current_scan_id = scan_count_.load();
        
        double dist_to_h1 = std::hypot(robot_x - human1_x_, robot_y - human1_y_);
        double dist_to_h2 = std::hypot(robot_x - human2_x_, robot_y - human2_y_);
        
        bool near_h1_original = (dist_to_h1 < near_threshold_);
        bool near_h2_original = (dist_to_h2 < near_threshold_);
        
        if (near_h1_original) scans_near_h1_++;
        if (near_h2_original) scans_near_h2_++;
        
        bool h1_hit_this_scan = false;
        bool h2_hit_this_scan = false;
        
        bool found_dynamic_obstacle = false;
        double dynamic_x = 0, dynamic_y = 0;

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float r = scan->ranges[i];
            if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) {
                continue;
            }

            float angle = scan->angle_min + i * scan->angle_increment;

            geometry_msgs::msg::PointStamped p_laser, p_map;
            p_laser.header = scan->header;
            p_laser.point.x = r * std::cos(angle);
            p_laser.point.y = r * std::sin(angle);
            p_laser.point.z = 0.0;

            tf2::doTransform(p_laser, p_map, tf);
            
            double px = p_map.point.x;
            double py = p_map.point.y;

            int mx, my;
            int8_t occ = -1;
            {
                std::lock_guard<std::mutex> lock(map_mutex_);
                if (worldToMap(px, py, mx, my)) {
                    int idx = my * map_.info.width + mx;
                    if (idx >= 0 && idx < (int)map_.data.size()) {
                        occ = map_.data[idx];
                    }
                }
            }

            bool is_free_space = (occ >= 0 && occ < 50);

            double d1 = std::hypot(px - human1_x_, py - human1_y_);
            if (near_h1_original && d1 <= detection_radius_) {
                h1_hit_this_scan = true;
            }

            double d2 = std::hypot(px - human2_x_, py - human2_y_);
            if (near_h2_original && d2 <= detection_radius_) {
                h2_hit_this_scan = true;
            }

            if (is_free_space && r < 3.0) {
                found_dynamic_obstacle = true;
                dynamic_x = px;
                dynamic_y = py;
                
                std::lock_guard<std::mutex> lock(cluster_mutex_);
                addPointToClusters(px, py);
            }
        }

        {
            std::lock_guard<std::mutex> lock(cluster_mutex_);
            
            int gx = static_cast<int>(std::floor(robot_x));
            int gy = static_cast<int>(std::floor(robot_y));
            auto key = std::make_pair(gx, gy);
            
            if (dynamic_detections_.find(key) == dynamic_detections_.end()) {
                dynamic_detections_[key] = {robot_x, robot_y, 0, 0};
            }
            
            dynamic_detections_[key].total_scans++;
            if (found_dynamic_obstacle) {
                dynamic_detections_[key].hit_scans++;
                dynamic_detections_[key].x = (dynamic_detections_[key].x + dynamic_x) / 2.0;
                dynamic_detections_[key].y = (dynamic_detections_[key].y + dynamic_y) / 2.0;
            }
            
            if (h1_hit_this_scan) {
                human1_close_range_hits_[current_scan_id] = true;
            }
            if (h2_hit_this_scan) {
                human2_close_range_hits_[current_scan_id] = true;
            }
        }

        scan_count_++;
    }

    bool worldToMap(double wx, double wy, int &mx, int &my) const {
        double res = map_.info.resolution;
        mx = static_cast<int>((wx - map_.info.origin.position.x) / res);
        my = static_cast<int>((wy - map_.info.origin.position.y) / res);
        return (mx >= 0 && my >= 0 && mx < (int)map_.info.width && my < (int)map_.info.height);
    }

    void addPointToClusters(double x, double y) {
        for (auto &c : clusters_) {
            double d = std::hypot(x - c.cx(), y - c.cy());
            if (d < cluster_dist_thresh_) {
                c.sum_x += x;
                c.sum_y += y;
                c.count++;
                return;
            }
        }
        Cluster c;
        c.sum_x = x;
        c.sum_y = y;
        c.count = 1;
        clusters_.push_back(c);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    auto detector = std::make_shared<HumanDetector>();
    Navigator navigator(true);

    std::thread spin_thread([&]() { rclcpp::spin(detector); });

    std::cout << "[Main] Waiting for map..." << std::endl;
    while (rclcpp::ok() && !detector->hasMap()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "[Main] Map received!" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    detector->resetDetection();

    auto init_pose = std::make_shared<geometry_msgs::msg::Pose>();
    init_pose->position.x = 2.12;
    init_pose->position.y = -21.3;
    init_pose->orientation.z = 0.7071;
    init_pose->orientation.w = 0.7071;

    navigator.SetInitialPose(init_pose);
    navigator.WaitUntilNav2Active();

    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    
    auto add_wp = [&](double x, double y, double yaw_deg) {
        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = "map";
        p.pose.position.x = x;
        p.pose.position.y = y;
        double rad = yaw_deg * M_PI / 180.0;
        p.pose.orientation.z = std::sin(rad / 2.0);
        p.pose.orientation.w = std::cos(rad / 2.0);
        waypoints.push_back(p);
    };

    std::cout << "[Main] Creating comprehensive coverage pattern..." << std::endl;
    
    std::vector<std::pair<double, double>> grid_pattern = {
        {2.0, -18.0}, {-2.0, -18.0}, {-6.0, -18.0}, {-10.0, -18.0},
        {-10.0, -12.0}, {-10.0, -6.0}, {-10.0, 0.0},
        {-6.0, -2.0}, {-3.0, -1.0}, {0.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {3.0, 0.0},
        {2.0, 2.0}, {3.0, 3.0}, {4.0, 2.0}, {5.0, 1.0}, {6.0, 2.0}, {6.0, 4.0}, 
        {5.0, 5.0}, {4.0, 4.0}, {3.0, 4.0}, {5.0, 3.0},  // Multi-angle (5, 3)
        {8.0, 4.0}, {10.0, 6.0}, {10.0, 10.0},
        {6.0, 10.0}, {2.0, 10.0}, {-2.0, 10.0}, {-6.0, 10.0}, {-10.0, 10.0},
        {-10.0, 12.0}, {-10.0, 14.0}, {-12.0, 13.0}, {-12.0, 15.0}, {-12.0, 17.0}, {-10.0, 16.0},
        {-6.0, 18.0}, {-2.0, 20.0}, {2.0, 22.0}, {6.0, 20.0}, {8.0, 18.0},
        {4.0, 16.0}, {0.0, 16.0}, {-4.0, 16.0}, {-8.0, 16.0},
    };
    
    std::cout << "[Main] Pattern: " << grid_pattern.size() << " waypoints" << std::endl;
    
    for (const auto& [x, y] : grid_pattern) {
        add_wp(x, y, 90.0);
    }
    
    std::vector<int> spin_at_waypoints;
    for (size_t i = 0; i < grid_pattern.size(); ++i) {
        double x = grid_pattern[i].first;
        double y = grid_pattern[i].second;
        
        if (std::hypot(x - 1.0, y + 1.0) < 0.5) {
            spin_at_waypoints.push_back(i);
            std::cout << "[Main] 360° at WP" << i << " (H1 original)" << std::endl;
        }
        if (std::hypot(x - 5.0, y - 3.0) < 0.5) {
            spin_at_waypoints.push_back(i);
            std::cout << "[Main] 360° at WP" << i << " (H1 new @ 5,3)" << std::endl;
        }
        if (std::hypot(x + 12.0, y - 15.0) < 0.5) {
            spin_at_waypoints.push_back(i);
            std::cout << "[Main] 360° at WP" << i << " (H2 original)" << std::endl;
        }
    }

    for (size_t i = 0; i < waypoints.size(); ++i) {
        auto goal = std::make_shared<geometry_msgs::msg::Pose>(waypoints[i].pose);
        std::cout << "[Main] -> WP" << (i+1) << "/" << waypoints.size() << std::endl;
        
        navigator.GoToPose(goal);
        while (rclcpp::ok() && !navigator.IsTaskComplete()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (navigator.GetResult() == rclcpp_action::ResultCode::SUCCEEDED) {
            std::cout << "[Main] ✓ WP" << (i+1) << std::endl;
            
            if (std::find(spin_at_waypoints.begin(), spin_at_waypoints.end(), i) 
                != spin_at_waypoints.end()) {
                std::cout << "[Main] Spinning 360°..." << std::endl;
                navigator.Spin();
                while (rclcpp::ok() && !navigator.IsTaskComplete()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
            
            if ((i+1) % 10 == 0) detector->printDebugInfo();
        } else {
            std::cerr << "[Main] ✗ Failed WP" << (i+1) << std::endl;
        }
    }

    std::cout << "\n========== SCAN COMPLETE ==========" << std::endl;
    detector->reportHumans();
    
    rclcpp::shutdown();
    if (spin_thread.joinable()) spin_thread.join();
    
    return 0;
}
