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

// =============================================================================================
// MRTP FINAL PROJECT: ROBUST HUMAN DETECTOR (DRIFT-PROOF)
// Features: 
// 1. High-Tolerance Wall Filtering (Fixes "Random Area" detections)
// 2. Density-Based Clustering (Fixes false positives)
// 3. Heuristic Search + Autonomous Coverage
// =============================================================================================

#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <map>
#include <algorithm>
#include <limits>
#include <iomanip>

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

struct DetectionCluster {
    double x, y;
    int count;
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

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local().reliable(),
            std::bind(&HumanDetector::mapCallback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&HumanDetector::scanCallback, this, std::placeholders::_1));

        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&HumanDetector::amclCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Drift-Proof Human Detector Initialized.");
    }

    bool hasMap() const { return have_map_.load(); }
    
    bool isHuman1Found() const { 
        return (scans_near_h1_ > 20 && ((double)h1_hits_ / scans_near_h1_) > 0.1); 
    }
    
    bool isHuman2Found() const { 
        return (scans_near_h2_ > 20 && ((double)h2_hits_ / scans_near_h2_) > 0.1); 
    }

    // [MRTP 9.1] Safe Map Access
    int8_t getMapValue(double x, double y) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!have_map_) return -1;
        int grid_x = static_cast<int>((x - map_.info.origin.position.x) / map_.info.resolution);
        int grid_y = static_cast<int>((y - map_.info.origin.position.y) / map_.info.resolution);
        if (grid_x < 0 || grid_x >= (int)map_.info.width || grid_y < 0 || grid_y >= (int)map_.info.height) return -1; 
        return map_.data[grid_y * map_.info.width + grid_x];
    }

    std::vector<geometry_msgs::msg::PoseStamped> generateCoveragePath() {
        std::vector<geometry_msgs::msg::PoseStamped> goals;
        std::lock_guard<std::mutex> lock(map_mutex_); 
        if (!have_map_) return goals;

        double step_size = 4.0; // Efficient coverage
        
        auto is_free_unsafe = [&](double wx, double wy) {
            int grid_x = static_cast<int>((wx - map_.info.origin.position.x) / map_.info.resolution);
            int grid_y = static_cast<int>((wy - map_.info.origin.position.y) / map_.info.resolution);
            if (grid_x < 0 || grid_x >= (int)map_.info.width || grid_y < 0 || grid_y >= (int)map_.info.height) return false; 
            return map_.data[grid_y * map_.info.width + grid_x] == 0;
        };

        for (double y = map_.info.origin.position.y + 2.0; y < (map_.info.origin.position.y + (map_.info.height * map_.info.resolution)); y += step_size) {
            for (double x = map_.info.origin.position.x + 2.0; x < (map_.info.origin.position.x + (map_.info.width * map_.info.resolution)); x += step_size) {
                // Ensure footprint clearance (Check cross pattern)
                if (is_free_unsafe(x, y) && is_free_unsafe(x + 0.4, y) && is_free_unsafe(x - 0.4, y) && is_free_unsafe(x, y + 0.4) && is_free_unsafe(x, y - 0.4)) {
                    geometry_msgs::msg::PoseStamped p;
                    p.header.frame_id = "map";
                    p.pose.position.x = x;
                    p.pose.position.y = y;
                    p.pose.orientation.w = 1.0;
                    goals.push_back(p);
                }
            }
        }

        // Nearest Neighbor Sort
        if (goals.empty()) return goals;
        double current_x = current_pose_.position.x;
        double current_y = current_pose_.position.y;
        std::vector<geometry_msgs::msg::PoseStamped> sorted_goals;
        std::vector<geometry_msgs::msg::PoseStamped> remaining = goals;

        while (!remaining.empty()) {
            auto nearest_it = remaining.begin();
            double min_dist = std::numeric_limits<double>::max();
            for (auto it = remaining.begin(); it != remaining.end(); ++it) {
                double d = std::hypot(it->pose.position.x - current_x, it->pose.position.y - current_y);
                if (d < min_dist) { min_dist = d; nearest_it = it; }
            }
            sorted_goals.push_back(*nearest_it);
            current_x = nearest_it->pose.position.x;
            current_y = nearest_it->pose.position.y;
            remaining.erase(nearest_it);
        }
        return sorted_goals;
    }

    void reportFindings() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        bool h1_found = isHuman1Found();
        bool h2_found = isHuman2Found();

        std::vector<DetectionCluster> candidates;
        
        // ---------------------------------------------------------
        // IMPROVED FILTERING LOGIC
        // ---------------------------------------------------------
        for (const auto& [coord, count] : dynamic_obstacles_) {
            // 1. Noise Filter: Ignore points with few hits
            if (count < 25) continue; 
            
            double wx = coord.first / 10.0; 
            double wy = coord.second / 10.0;

            // 2. Original Location Filter
            if (h1_found && std::hypot(wx - H1_X, wy - H1_Y) < 1.5) continue;
            if (h2_found && std::hypot(wx - H2_X, wy - H2_Y) < 1.5) continue;

            // 3. HARD BOUNDS FILTER (Fixes your corner issue)
            // Warehouse is roughly -13 to +13 X, and -23 to +23 Y. 
            // Anything outside this box is a map artifact.
            if (wx < -13.0 || wx > 13.0 || wy < -22.0 || wy > 22.0) continue;

            candidates.push_back({wx, wy, count});
        }

        // 4. Clustering (Merge nearby points)
        std::vector<DetectionCluster> final_humans;
        for (const auto& cand : candidates) {
            bool merged = false;
            for (auto& f : final_humans) {
                // If within 1.0m, assume same object
                if (std::hypot(f.x - cand.x, f.y - cand.y) < 1.0) {
                    // Weighted average for center
                    double total = f.count + cand.count;
                    f.x = (f.x * f.count + cand.x * cand.count) / total;
                    f.y = (f.y * f.count + cand.y * cand.count) / total;
                    f.count += cand.count;
                    merged = true;
                    break;
                }
            }
            if (!merged) final_humans.push_back(cand);
        }

        std::sort(final_humans.begin(), final_humans.end(), 
            [](const DetectionCluster &a, const DetectionCluster &b) { return a.count > b.count; });

        std::cout << "\n========================================" << std::endl;
        std::cout << "       HUMAN DETECTION REPORT" << std::endl;
        std::cout << "========================================" << std::endl;

        if (h1_found) {
            std::cout << "[Human 1] STILL AT ORIGINAL (" << H1_X << ", " << H1_Y << ") [DETECTED]" << std::endl;
        } else {
            std::cout << "[Human 1] MOVED from (" << H1_X << ", " << H1_Y << ")" << std::endl;
            if (!final_humans.empty()) {
                std::cout << "          NEW LOCATION: (" << final_humans[0].x << ", " << final_humans[0].y << ")" << std::endl;
            } else {
                std::cout << "          NEW LOCATION: Not found (Check scan coverage)" << std::endl;
            }
        }

        if (h2_found) {
            std::cout << "[Human 2] STILL AT ORIGINAL (" << H2_X << ", " << H2_Y << ") [DETECTED]" << std::endl;
        } else {
            std::cout << "[Human 2] MOVED from (" << H2_X << ", " << H2_Y << ")" << std::endl;
            size_t idx = (!h1_found && final_humans.size() > 1) ? 1 : 0;
            if (final_humans.size() > idx) {
                std::cout << "          NEW LOCATION: (" << final_humans[idx].x << ", " << final_humans[idx].y << ")" << std::endl;
            } else {
                std::cout << "          NEW LOCATION: Not found (Check scan coverage)" << std::endl;
            }
        }
        std::cout << "========================================\n" << std::endl;
    }

    const double H1_X = 1.0, H1_Y = -1.0;
    const double H2_X = -12.0, H2_Y = 15.0;

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

    nav_msgs::msg::OccupancyGrid map_;
    std::atomic<bool> have_map_{false};
    std::atomic<bool> is_localized_{false};
    
    std::mutex map_mutex_;
    std::mutex data_mutex_;
    std::mutex pose_mutex_;
    
    geometry_msgs::msg::Pose current_pose_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    int scans_near_h1_{0}, h1_hits_{0};
    int scans_near_h2_{0}, h2_hits_{0};

    std::map<std::pair<int,int>, int> dynamic_obstacles_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_ = *msg;
        have_map_.store(true);
    }

    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_ = msg->pose.pose;
        is_localized_.store(true);
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        if (!have_map_.load() || !is_localized_.load()) return;

        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_.lookupTransform("map", scan->header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) { return; }

        double rx = tf.transform.translation.x;
        double ry = tf.transform.translation.y;

        // Proximity checks for original locations
        bool near_h1 = std::hypot(rx - H1_X, ry - H1_Y) < 4.0;
        bool near_h2 = std::hypot(rx - H2_X, ry - H2_Y) < 4.0;

        if (near_h1) scans_near_h1_++;
        if (near_h2) scans_near_h2_++;

        bool h1_hit_this_scan = false;
        bool h2_hit_this_scan = false;

        auto get_map_val_unsafe = [&](double x, double y) -> int {
            int gx = static_cast<int>((x - map_.info.origin.position.x) / map_.info.resolution);
            int gy = static_cast<int>((y - map_.info.origin.position.y) / map_.info.resolution);
            if (gx < 0 || gx >= (int)map_.info.width || gy < 0 || gy >= (int)map_.info.height) return -1;
            return map_.data[gy * map_.info.width + gx];
        };

        std::lock_guard<std::mutex> lock(map_mutex_);

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float r = scan->ranges[i];
            
            // Limit detection range to 5.0m to reduce far-field noise
            if (!std::isfinite(r) || r < scan->range_min || r > 5.0) continue;

            float angle = scan->angle_min + i * scan->angle_increment;
            geometry_msgs::msg::PointStamped p_laser, p_map;
            p_laser.header = scan->header;
            p_laser.point.x = r * std::cos(angle);
            p_laser.point.y = r * std::sin(angle);
            p_laser.point.z = 0.0;

            tf2::doTransform(p_laser, p_map, tf);
            double wx = p_map.point.x;
            double wy = p_map.point.y;

            if (near_h1 && std::hypot(wx - H1_X, wy - H1_Y) < 0.5) h1_hit_this_scan = true;
            if (near_h2 && std::hypot(wx - H2_X, wy - H2_Y) < 0.5) h2_hit_this_scan = true;

            // -----------------------------------------------------
            // AGGRESSIVE WALL FILTERING
            // -----------------------------------------------------
            int map_val = get_map_val_unsafe(wx, wy);
            
            bool wall_nearby = false;
            int gx = static_cast<int>((wx - map_.info.origin.position.x) / map_.info.resolution);
            int gy = static_cast<int>((wy - map_.info.origin.position.y) / map_.info.resolution);
            
            // Check radius of 10 cells (~30-50cm depending on map res)
            // This is the key fix for "phantom" detections near walls due to drift.
            int check_rad = 10; 
            
            for(int dy=-check_rad; dy<=check_rad && !wall_nearby; ++dy) {
                for(int dx=-check_rad; dx<=check_rad && !wall_nearby; ++dx) {
                    int idx = (gy + dy) * map_.info.width + (gx + dx);
                    // Check bounds and if occupied
                    if (idx >= 0 && idx < (int)map_.data.size() && map_.data[idx] == 100) {
                        wall_nearby = true;
                    }
                }
            }

            // Only count if:
            // 1. Map says FREE
            // 2. NO WALLS within ~30cm
            // 3. Inside reasonable laser range
            if (map_val == 0 && !wall_nearby) {
                std::lock_guard<std::mutex> data_lock(data_mutex_);
                int key_x = static_cast<int>(std::round(wx * 10));
                int key_y = static_cast<int>(std::round(wy * 10));
                dynamic_obstacles_[{key_x, key_y}]++;
            }
        }

        if (h1_hit_this_scan) h1_hits_++;
        if (h2_hit_this_scan) h2_hits_++;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto detector = std::make_shared<HumanDetector>();
    Navigator navigator(true);
    std::thread spin_thread([&]() { rclcpp::spin(detector); });

    auto start_time = std::chrono::high_resolution_clock::now();

    // 1. Wait for Map
    std::cout << "\n[Main] Waiting for Map..." << std::endl;
    while (rclcpp::ok() && !detector->hasMap()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "[Main] Map Received." << std::endl;

    // 2. Initialize Pose
    auto init_pose = std::make_shared<geometry_msgs::msg::Pose>();
    init_pose->position.x = 2.12;
    init_pose->position.y = -21.3;
    init_pose->orientation.z = 0.7071;
    init_pose->orientation.w = 0.7071;
    
    std::cout << "[Main] Initializing Pose..." << std::endl;
    navigator.SetInitialPose(init_pose);
    navigator.WaitUntilNav2Active();

    // ---------------------------------------------------------
    // OPTIMIZATION: Check Original Locations First
    // ---------------------------------------------------------
    std::cout << "\n[Main] OPTIMIZATION: Checking original human locations first..." << std::endl;
    
    // Check H1
    auto h1_goal = std::make_shared<geometry_msgs::msg::Pose>();
    h1_goal->position.x = detector->H1_X + 1.5; 
    h1_goal->position.y = detector->H1_Y;
    h1_goal->orientation.w = 1.0;
    std::cout << "[Main] Checking Human 1..." << std::endl;
    navigator.GoToPose(h1_goal);
    while (rclcpp::ok() && !navigator.IsTaskComplete()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
    navigator.Spin(); 
    while (rclcpp::ok() && !navigator.IsTaskComplete()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }

    // Check H2
    auto h2_goal = std::make_shared<geometry_msgs::msg::Pose>();
    h2_goal->position.x = detector->H2_X + 1.5; 
    h2_goal->position.y = detector->H2_Y;
    h2_goal->orientation.w = 1.0;
    std::cout << "[Main] Checking Human 2..." << std::endl;
    navigator.GoToPose(h2_goal);
    while (rclcpp::ok() && !navigator.IsTaskComplete()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
    navigator.Spin();
    while (rclcpp::ok() && !navigator.IsTaskComplete()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }

    // Verify presence
    bool h1_ok = detector->isHuman1Found();
    bool h2_ok = detector->isHuman2Found();

    if (h1_ok && h2_ok) {
        std::cout << "[Main] SUCCESS: Both humans found at original locations!" << std::endl;
    } else {
        std::cout << "[Main] WARNING: One or more humans moved. Starting Autonomous Coverage..." << std::endl;
        
        // ---------------------------------------------------------
        // FALLBACK: Autonomous Coverage
        // ---------------------------------------------------------
        auto waypoints = detector->generateCoveragePath();
        int wp_count = 0;
        for (const auto& wp : waypoints) {
            wp_count++;
            if (std::hypot(wp.pose.position.x - 2.12, wp.pose.position.y - (-21.3)) < 2.0) continue;

            std::cout << "[Main] Navigating to WP " << wp_count << "/" << waypoints.size() << "..." << std::flush;
            auto goal = std::make_shared<geometry_msgs::msg::Pose>(wp.pose);
            navigator.GoToPose(goal);
            while (rclcpp::ok() && !navigator.IsTaskComplete()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
            if (navigator.GetResult() == rclcpp_action::ResultCode::SUCCEEDED) {
                std::cout << " DONE" << std::endl;
            } else {
                std::cout << " SKIPPED" << std::endl;
            }
        }
    }

    // STOP TIMER (Calculated in Minutes and Seconds)
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_sec = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    
    int total_seconds = static_cast<int>(duration_sec.count());
    int minutes = total_seconds / 60;
    int seconds = total_seconds % 60;

    // 5. Final Report
    detector->reportFindings();
    std::cout << "[Main] Mission Completed in " << minutes << " min " << seconds << " sec." << std::endl;

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}