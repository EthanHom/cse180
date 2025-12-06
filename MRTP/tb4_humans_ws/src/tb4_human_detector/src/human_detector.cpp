// run the project in terminal: 
/* 

FOR DOCKER

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
// MRTP FINAL PROJECT: ROBUST HUMAN DETECTOR v10 (High Fidelity)
// 
// Compliant with:
// - MRTP Ch 4 (TF2 Transformations)
// - MRTP Ch 6 (Nav2 Planning & Heuristics)
// - MRTP Ch 7 & 9 (Occupancy Grid & LaserScan Fusion)
// 
// Fixes:
// 1. "Still at Original" False Positives -> Fixed by resetting counters & higher threshold.
// 2. "Ghost Walls" -> Fixed by balanced wall buffering (12 cells).
// 3. "Not Found" -> Fixed by allowing map bounds up to +/- 14.8m.
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

// ---------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------
struct DetectionCluster {
    double x, y;
    int count;
};

struct Candidate {
    double x = 0.0;
    double y = 0.0;
};

class HumanDetector : public rclcpp::Node {
public:
    HumanDetector()
        : Node("tb4_human_detector"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        // [MRTP 5.4.3] Simulation time is mandatory for TF sync
        if (!this->has_parameter("use_sim_time")) {
            this->declare_parameter("use_sim_time", true);
        }
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        // [MRTP 9.5]
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local().reliable(),
            std::bind(&HumanDetector::mapCallback, this, std::placeholders::_1));

        // [MRTP 7.4.1]
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&HumanDetector::scanCallback, this, std::placeholders::_1));

        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&HumanDetector::amclCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "High-Fidelity Human Detector Initialized.");
    }

    bool hasMap() const { return have_map_.load(); }
    
    // Call this RIGHT BEFORE checking the specific location to clear old data
    void resetCounters() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        scans_near_h1_ = 0; h1_hits_ = 0;
        scans_near_h2_ = 0; h2_hits_ = 0;
    }

    bool isHuman1AtStart() const { 
        // [FIX] Increased threshold to 25% hit rate to rule out noise
        if (scans_near_h1_ < 10) return false;
        return (((double)h1_hits_ / scans_near_h1_) > 0.25); 
    }
    
    bool isHuman2AtStart() const { 
        if (scans_near_h2_ < 10) return false;
        return (((double)h2_hits_ / scans_near_h2_) > 0.25); 
    }

    // [MRTP 6 & 9] Heuristic Candidate Detection
    Candidate getBestNewCandidate() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        std::vector<DetectionCluster> candidates;
        for (const auto& [coord, count] : dynamic_obstacles_) {
            // Threshold: Need 40 hits to confirm it's not sensor noise
            if (count < 40) continue; 
            
            double wx = coord.first / 10.0; 
            double wy = coord.second / 10.0;

            // Ignore original spots (handled separately)
            if (std::hypot(wx - H1_X, wy - H1_Y) < 2.0) continue;
            if (std::hypot(wx - H2_X, wy - H2_Y) < 2.0) continue;
            
            // [FIX] Bounds: +/- 14.8 to catch corners but ignore outside void
            if (wx < -14.8 || wx > 14.8 || wy < -24.8 || wy > 24.8) continue;

            candidates.push_back({wx, wy, count});
        }

        // Sort by confidence (hit count)
        std::sort(candidates.begin(), candidates.end(), 
            [](const DetectionCluster &a, const DetectionCluster &b) { return a.count > b.count; });

        if (candidates.empty()) return {0.0, 0.0};
        return {candidates[0].x, candidates[0].y};
    }
    
    // Clears specific area (used before verification spin)
    void clearCandidateArea(double x, double y) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (auto it = dynamic_obstacles_.begin(); it != dynamic_obstacles_.end();) {
            double wx = it->first.first / 10.0;
            double wy = it->first.second / 10.0;
            if (std::hypot(wx - x, wy - y) < 2.0) {
                it = dynamic_obstacles_.erase(it);
            } else {
                ++it;
            }
        }
    }

    // [MRTP 6] Map-Based Path Planning
    std::vector<geometry_msgs::msg::PoseStamped> generateCoveragePath() {
        std::vector<geometry_msgs::msg::PoseStamped> goals;
        std::lock_guard<std::mutex> lock(map_mutex_); 
        if (!have_map_) return goals;

        // 3.5m step allows looking deep into corners with 6.0m laser range
        double step_size = 3.5; 
        
        auto is_free_unsafe = [&](double wx, double wy) {
            int grid_x = static_cast<int>((wx - map_.info.origin.position.x) / map_.info.resolution);
            int grid_y = static_cast<int>((wy - map_.info.origin.position.y) / map_.info.resolution);
            if (grid_x < 0 || grid_x >= (int)map_.info.width || grid_y < 0 || grid_y >= (int)map_.info.height) return false; 
            return map_.data[grid_y * map_.info.width + grid_x] == 0;
        };

        for (double y = map_.info.origin.position.y + 2.0; y < (map_.info.origin.position.y + (map_.info.height * map_.info.resolution)); y += step_size) {
            for (double x = map_.info.origin.position.x + 2.0; x < (map_.info.origin.position.x + (map_.info.width * map_.info.resolution)); x += step_size) {
                if (is_free_unsafe(x, y) && is_free_unsafe(x + 0.3, y) && is_free_unsafe(x - 0.3, y)) {
                    geometry_msgs::msg::PoseStamped p;
                    p.header.frame_id = "map";
                    p.pose.position.x = x;
                    p.pose.position.y = y;
                    p.pose.orientation.w = 1.0;
                    goals.push_back(p);
                }
            }
        }

        // Sort Nearest Neighbor
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

    void reportFindings(const std::vector<Candidate>& verified_locs) {
        bool h1_found = isHuman1AtStart();
        bool h2_found = isHuman2AtStart();

        std::cout << "\n========================================" << std::endl;
        std::cout << "       HUMAN DETECTION REPORT" << std::endl;
        std::cout << "========================================" << std::endl;

        if (h1_found) {
            std::cout << "[Human 1] STILL AT ORIGINAL (" << H1_X << ", " << H1_Y << ") [DETECTED]" << std::endl;
        } else {
            std::cout << "[Human 1] MOVED from (" << H1_X << ", " << H1_Y << ")" << std::endl;
            if (!verified_locs.empty()) {
                std::cout << "          NEW LOCATION: (" << verified_locs[0].x << ", " << verified_locs[0].y << ")" << std::endl;
            } else {
                std::cout << "          NEW LOCATION: Not found" << std::endl;
            }
        }

        if (h2_found) {
            std::cout << "[Human 2] STILL AT ORIGINAL (" << H2_X << ", " << H2_Y << ") [DETECTED]" << std::endl;
        } else {
            std::cout << "[Human 2] MOVED from (" << H2_X << ", " << H2_Y << ")" << std::endl;
            size_t idx = (!h1_found && verified_locs.size() > 1) ? 1 : 0;
            if (verified_locs.size() > idx) {
                std::cout << "          NEW LOCATION: (" << verified_locs[idx].x << ", " << verified_locs[idx].y << ")" << std::endl;
            } else {
                std::cout << "          NEW LOCATION: Not found" << std::endl;
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
            // [MRTP 4.13] TF Tree lookup
            tf = tf_buffer_.lookupTransform("map", scan->header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) { return; }

        double rx = tf.transform.translation.x;
        double ry = tf.transform.translation.y;

        // Proximity checks (Robot must be close for valid check)
        bool near_h1 = std::hypot(rx - H1_X, ry - H1_Y) < 3.0;
        bool near_h2 = std::hypot(rx - H2_X, ry - H2_Y) < 3.0;

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
            if (!std::isfinite(r) || r < scan->range_min || r > 5.0) continue; 

            float angle = scan->angle_min + i * scan->angle_increment;
            geometry_msgs::msg::PointStamped p_laser, p_map;
            p_laser.header = scan->header;
            p_laser.point.x = r * std::cos(angle);
            p_laser.point.y = r * std::sin(angle);
            p_laser.point.z = 0.0;

            // [MRTP 4] Coordinate Transform
            tf2::doTransform(p_laser, p_map, tf);
            double wx = p_map.point.x;
            double wy = p_map.point.y;

            // Check direct hits on original spots (0.6m radius = approx human width)
            if (near_h1 && std::hypot(wx - H1_X, wy - H1_Y) < 0.6) h1_hit_this_scan = true;
            if (near_h2 && std::hypot(wx - H2_X, wy - H2_Y) < 0.6) h2_hit_this_scan = true;

            // [MRTP 9] Map vs Sensor Check
            int map_val = get_map_val_unsafe(wx, wy);
            bool wall_nearby = false;
            int gx = static_cast<int>((wx - map_.info.origin.position.x) / map_.info.resolution);
            int gy = static_cast<int>((wy - map_.info.origin.position.y) / map_.info.resolution);
            
            // [FIX] BALANCED WALL BUFFER
            // 12 cells * 0.03m = 36cm. 
            // Handles drift but allows detecting humans standing near shelves.
            int check_rad = 12; 
            for(int dy=-check_rad; dy<=check_rad && !wall_nearby; ++dy) {
                for(int dx=-check_rad; dx<=check_rad && !wall_nearby; ++dx) {
                    int idx = (gy + dy) * map_.info.width + (gx + dx);
                    if (idx >= 0 && idx < (int)map_.data.size()) {
                        int8_t val = map_.data[idx];
                        if (val > 50 || val == -1) {
                            wall_nearby = true;
                        }
                    }
                }
            }

            if (map_val == 0 && !wall_nearby && r < 4.5) {
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

    std::cout << "\n[Main] Waiting for Map..." << std::endl;
    while (rclcpp::ok() && !detector->hasMap()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
    std::cout << "[Main] Map Received." << std::endl;

    auto init_pose = std::make_shared<geometry_msgs::msg::Pose>();
    init_pose->position.x = 2.12; init_pose->position.y = -21.3;
    init_pose->orientation.z = 0.7071; init_pose->orientation.w = 0.7071;
    navigator.SetInitialPose(init_pose);
    navigator.WaitUntilNav2Active();

    // ---------------------------------------------------------
    // PHASE 1: CHECK ORIGINAL SPOTS (Optimized)
    // ---------------------------------------------------------
    std::cout << "\n[Main] Phase 1: Checking original locations..." << std::endl;
    
    // [FIX] Reset counters before measurement to ensure clean data
    detector->resetCounters();

    // Check H1
    auto h1_goal = std::make_shared<geometry_msgs::msg::Pose>();
    // [FIX] Drive closer (1.0m) for accurate laser reading
    h1_goal->position.x = detector->H1_X + 1.0; 
    h1_goal->position.y = detector->H1_Y; 
    h1_goal->orientation.w = 1.0;
    navigator.GoToPose(h1_goal);
    while (rclcpp::ok() && !navigator.IsTaskComplete()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
    navigator.Spin(); 
    while (rclcpp::ok() && !navigator.IsTaskComplete()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }

    // Check H2
    auto h2_goal = std::make_shared<geometry_msgs::msg::Pose>();
    h2_goal->position.x = detector->H2_X + 1.0; 
    h2_goal->position.y = detector->H2_Y; 
    h2_goal->orientation.w = 1.0;
    navigator.GoToPose(h2_goal);
    while (rclcpp::ok() && !navigator.IsTaskComplete()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
    navigator.Spin();
    while (rclcpp::ok() && !navigator.IsTaskComplete()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }

    bool h1_found = detector->isHuman1AtStart();
    bool h2_found = detector->isHuman2AtStart();
    int missing_count = (h1_found ? 0 : 1) + (h2_found ? 0 : 1);

    std::cout << "[Main] Status: H1@" << (h1_found ? "Start" : "Moved") 
              << ", H2@" << (h2_found ? "Start" : "Moved") << std::endl;

    // ---------------------------------------------------------
    // PHASE 2: VERIFIED SEARCH
    // ---------------------------------------------------------
    std::vector<Candidate> verified_locations;

    if (missing_count == 0) {
        std::cout << "[Main] SUCCESS: Both humans found at original locations!" << std::endl;
    } 
    else {
        std::cout << "[Main] WARNING: Need to find " << missing_count << " more human(s). Starting Search..." << std::endl;
        
        auto waypoints = detector->generateCoveragePath();
        int wp_count = 0;

        for (const auto& wp : waypoints) {
            if (verified_locations.size() >= static_cast<size_t>(missing_count)) break;

            Candidate cand = detector->getBestNewCandidate();
            
            if (cand.x != 0.0) {
                bool already_checked = false;
                for(auto loc : verified_locations) {
                    if(std::hypot(cand.x - loc.x, cand.y - loc.y) < 2.0) already_checked = true;
                }

                if (!already_checked) {
                    std::cout << "\n[Main] INTERRUPT: Investigating candidate at (" << cand.x << ", " << cand.y << ")..." << std::endl;
                    navigator.CancelTask(); 
                    
                    auto cand_goal = std::make_shared<geometry_msgs::msg::Pose>();
                    cand_goal->position.x = cand.x; 
                    cand_goal->position.y = cand.y; 
                    cand_goal->orientation.w = 1.0;

                    navigator.GoToPose(cand_goal);
                    while (rclcpp::ok() && !navigator.IsTaskComplete()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
                    
                    // [FIX] Clear this candidate's old data so we verify based on FRESH scan
                    detector->clearCandidateArea(cand.x, cand.y);
                    
                    navigator.Spin(); 
                    while (rclcpp::ok() && !navigator.IsTaskComplete()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }

                    // Now check if it re-appeared (confirmed)
                    Candidate confirmed = detector->getBestNewCandidate();
                    
                    if (confirmed.x != 0.0 && std::hypot(confirmed.x - cand.x, confirmed.y - cand.y) < 1.5) {
                        std::cout << "[Main] CONFIRMED HUMAN FOUND!" << std::endl;
                        verified_locations.push_back(cand);
                    } else {
                        std::cout << "[Main] False positive (Ghost/Wall). Clearing and resuming..." << std::endl;
                        detector->clearCandidateArea(cand.x, cand.y);
                    }
                    continue; 
                }
            }

            wp_count++;
            if (std::hypot(wp.pose.position.x - 2.12, wp.pose.position.y - (-21.3)) < 2.0) continue;

            std::cout << "[Main] Navigating to WP " << wp_count << "/" << waypoints.size() << "..." << std::flush;
            auto goal = std::make_shared<geometry_msgs::msg::Pose>(wp.pose);
            navigator.GoToPose(goal);
            
            while (rclcpp::ok() && !navigator.IsTaskComplete()) { 
                Candidate mid_cand = detector->getBestNewCandidate();
                if (mid_cand.x != 0.0) {
                     bool known = false;
                     for(auto loc : verified_locations) if(std::hypot(mid_cand.x - loc.x, mid_cand.y - loc.y) < 2.0) known = true;
                     if(!known) {
                         navigator.CancelTask();
                         break; 
                     }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
            }

            if (navigator.GetResult() == rclcpp_action::ResultCode::SUCCEEDED) {
                std::cout << " DONE" << std::endl;
            } else {
                std::cout << " INTERRUPTED/SKIPPED" << std::endl;
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_sec = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    int total_seconds = static_cast<int>(duration_sec.count());
    int minutes = total_seconds / 60;
    int seconds = total_seconds % 60;

    detector->reportFindings(verified_locations);
    std::cout << "[Main] Mission Completed in " << minutes << " min " << seconds << " sec." << std::endl;

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}