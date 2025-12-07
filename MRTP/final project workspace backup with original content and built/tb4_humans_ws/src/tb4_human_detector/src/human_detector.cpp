// run the project in terminal: 
/* 

FOR DOCKER on Mac

    Terminal 1:
        make vnc
        make bash
        cd /MRTP/MRTP/
            optional: colcon build
        source install/setup.bash
        ros2 launch gazeboenvs tb4_warehouse.launch.py use_rviz:=true

    Terminal 2:
        make shell
        cd /MRTP/tb4_humans_ws/
        colcon build
        source install/setup.bash
            note: sometimes you have to source install/setup.bash in the 
                    /MRTP/MRTP folder first, then cd back to the /MRTP/tb4_humans_ws 
                    folder and also colcon build and source install/setup.bash 
        ros2 run tb4_human_detector human_detector_node 

*/


// includes for C++ containers and ROS2 libraries
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

// struct for a detection cluster / manage detection candidates
struct DetectionCluster {
    double x, y;
    int count;
};

struct Candidate {
    double x = 0.0;
    double y = 0.0;
};

// ROS2 Node Class for the Human Detector
class HumanDetector : public rclcpp::Node {
public:
    HumanDetector()
        : Node("tb4_human_detector"),   // node name
          tf_buffer_(this->get_clock()),    // tf2 buffer for transforming between frames
          tf_listener_(tf_buffer_)
    {
        // from Chapter 5, parameter declaration
        if (!this->has_parameter("use_sim_time")) {
            this->declare_parameter("use_sim_time", true);
        }
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        
        // subscribe to /map (Chapter 6/8) using QoS for reliability
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local().reliable(),
            std::bind(&HumanDetector::mapCallback, this, std::placeholders::_1));

        // subscribe to /scan (Chapter 7)
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&HumanDetector::scanCallback, this, std::placeholders::_1));

        // subscribe to AMCL pose (Chapter 9) to know where robot is
        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&HumanDetector::amclCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Final Human Detector Ready.");
    }

    // Helper to check if map is received
    bool hasMap() const { return have_map_.load(); }
    
    // Call this to clear data - clears detection counters for a fresh measurement
    void resetCounters() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        scans_near_h1_ = 0; h1_hits_ = 0;
        scans_near_h2_ = 0; h2_hits_ = 0;
    }

    // if >30% of laser scans hit something at the start locations, human is there
    // threshold to 30% to avoid false positives from noise
    bool isHuman1AtStart() const { 
        if (scans_near_h1_ < 10) return false;
        return (((double)h1_hits_ / scans_near_h1_) > 0.30); 
    }
    
    bool isHuman2AtStart() const { 
        if (scans_near_h2_ < 10) return false;
        return (((double)h2_hits_ / scans_near_h2_) > 0.30); 
    }

    // iterates over `dynamic_obstacles_` map to find the cluster with the most hits
    Candidate getBestNewCandidate() {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Filters out points with low confidence (<30 hits) or out of bounds)
        // Returns the X,Y of the highest confidence unknown object
        
        std::vector<DetectionCluster> candidates;
        for (const auto& [coord, count] : dynamic_obstacles_) {
            // Threshold: 30 hits ensures solid object
            if (count < 30) continue; 
            
            double wx = coord.first / 10.0; 
            double wy = coord.second / 10.0;

            // Ignore original spots
            if (std::hypot(wx - H1_X, wy - H1_Y) < 2.0) continue;
            if (std::hypot(wx - H2_X, wy - H2_Y) < 2.0) continue;
            
            // if (wx < -14.5 || wx > 14.5 || wy < -24.5 || wy > 24.5) continue;  // prone to false positives
            // if (wx < -14 || wx > 14 || wy < -24 || wy > 24) continue;    // doesn't get humans close to wall
            if (wx < -14.25 || wx > 14.25 || wy < -24.25 || wy > 24.25) continue;   // good boundary

            candidates.push_back({wx, wy, count});
        }

        std::sort(candidates.begin(), candidates.end(), 
            [](const DetectionCluster &a, const DetectionCluster &b) { return a.count > b.count; });

        if (candidates.empty()) return {0.0, 0.0};
        return {candidates[0].x, candidates[0].y};
    }
    
    // Removes a candidate from the map after we have visited/verified it
    void clearCandidate(double x, double y) {
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

    // Helper to read occupancy grid data at a specific (x,y)
    int8_t getMapValue(double x, double y) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!have_map_) return -1;
        int grid_x = static_cast<int>((x - map_.info.origin.position.x) / map_.info.resolution);
        int grid_y = static_cast<int>((y - map_.info.origin.position.y) / map_.info.resolution);
        if (grid_x < 0 || grid_x >= (int)map_.info.width || grid_y < 0 || grid_y >= (int)map_.info.height) return -1; 
        return map_.data[grid_y * map_.info.width + grid_x];
    }

    // Generates a "lawnmower" path (waypoints) to cover the map
    std::vector<geometry_msgs::msg::PoseStamped> generateCoveragePath() {

        // ... (Iterates x/y with step_size 3.5m) ...
        // ... (Checks if waypoints are in free space using `is_free_unsafe`) ...
        // Sorts points to visit the nearest one next (Greedy Path Planning).

        std::vector<geometry_msgs::msg::PoseStamped> goals;
        std::lock_guard<std::mutex> lock(map_mutex_); 
        if (!have_map_) return goals;

        double step_size = 3.5; 
        
        // Helper to check if a point is in free space
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

        // Sort goals to visit the nearest one next (Greedy Path Planning)
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

    // Prints the final results to the terminal.
    void reportFindings(bool h1_found, bool h2_found, const std::vector<Candidate>& verified_locs) {
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

    // Constants for the human locations
    const double H1_X = 1.0, H1_Y = -1.0;
    const double H2_X = -12.0, H2_Y = 15.0;

private:
    // Member variables for subscriptions, data storage, and mutexes
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

    // `dynamic_obstacles_` maps grid coordinates to hit counts
    std::map<std::pair<int,int>, int> dynamic_obstacles_;

    // Callbacks to store data
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_ = *msg;
        have_map_.store(true);
    }

    // Callback to store AMCL pose
    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_ = msg->pose.pose;
        is_localized_.store(true);
    }

    // Detection
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        if (!have_map_.load() || !is_localized_.load()) return;

        // Get transform from "map" to laser frame (Chapter 4)
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_.lookupTransform("map", scan->header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) { return; }

        // Check if robot is physically near the starting spots H1/H2

        // Calculate the distance between the robot and the starting spots
        double rx = tf.transform.translation.x;
        double ry = tf.transform.translation.y;

        // Check if robot is physically near the starting spots H1/H2
        bool near_h1 = std::hypot(rx - H1_X, ry - H1_Y) < 3.5;
        bool near_h2 = std::hypot(rx - H2_X, ry - H2_Y) < 3.5;

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

        // Iterate over laser ranges (Chapter 7)
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            
            // Convert polar (range/angle) to Cartesian (x/y)
            // Transform point to Map Frame using tf2::doTransform
            
            float r = scan->ranges[i];

            // Skip invalid ranges
            if (!std::isfinite(r) || r < scan->range_min || r > 5.5) continue; 

            float angle = scan->angle_min + i * scan->angle_increment;
            geometry_msgs::msg::PointStamped p_laser, p_map;
            p_laser.header = scan->header;
            p_laser.point.x = r * std::cos(angle);
            p_laser.point.y = r * std::sin(angle);
            p_laser.point.z = 0.0;

            tf2::doTransform(p_laser, p_map, tf);
            double wx = p_map.point.x;
            double wy = p_map.point.y;

            // Check if point hits H1/H2 original location
            if (near_h1 && std::hypot(wx - H1_X, wy - H1_Y) < 0.5) h1_hit_this_scan = true;
            if (near_h2 && std::hypot(wx - H2_X, wy - H2_Y) < 0.5) h2_hit_this_scan = true;

            int map_val = get_map_val_unsafe(wx, wy);
            bool wall_nearby = false;
            int gx = static_cast<int>((wx - map_.info.origin.position.x) / map_.info.resolution);
            int gy = static_cast<int>((wy - map_.info.origin.position.y) / map_.info.resolution);
            
            // Strong Wall Buffer: 18 cells (54cm). 
            // Ensures we do not detect walls as humans even with drift.
            int check_rad = 18; 
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

            // "Background Subtraction":
            // 1. Get map value at hit location
            // 2. Check nearby cells for walls (buffer)
            // 3. If map says FREE (0) but laser says HIT, increment dynamic_obstacle count
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
    // initialize navigator wrapper (chapter 6)
    Navigator navigator(true);

    // spin the detector node in a separate/background thread so callbacks process
    std::thread spin_thread([&]() { rclcpp::spin(detector); });

    auto start_time = std::chrono::high_resolution_clock::now();

    std::cout << "\n[Main] Waiting for Map..." << std::endl;
    while (rclcpp::ok() && !detector->hasMap()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
    std::cout << "[Main] Map Received." << std::endl;

    // set initial pose for AMCL
    auto init_pose = std::make_shared<geometry_msgs::msg::Pose>();
    init_pose->position.x = 2.12; init_pose->position.y = -21.3;
    init_pose->orientation.z = 0.7071; init_pose->orientation.w = 0.7071;
    // set initial pose
    navigator.SetInitialPose(init_pose);
    navigator.WaitUntilNav2Active();

    // ---------------------------------------------------------
    // PHASE 1: CHECK ORIGINAL SPOTS
    // ---------------------------------------------------------
    std::cout << "\n[Main] Phase 1: Checking original locations..." << std::endl;
    
    // Logic: Move, then Reset Counters, then Spin & Measure
    auto h1_goal = std::make_shared<geometry_msgs::msg::Pose>();
    h1_goal->position.x = detector->H1_X + 1.0; 
    h1_goal->position.y = detector->H1_Y; 
    h1_goal->orientation.w = 1.0;
    // send robot to H1
    navigator.GoToPose(h1_goal);
    while (rclcpp::ok() && !navigator.IsTaskComplete()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
    
    // measure hits at H1
    detector->resetCounters(); // Reset AFTER arrival
    navigator.Spin();   // spin robot to update laser scan
    // wait
    while (rclcpp::ok() && !navigator.IsTaskComplete()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
    
    bool h1_found = detector->isHuman1AtStart();

    // repeat for H2

    auto h2_goal = std::make_shared<geometry_msgs::msg::Pose>();
    h2_goal->position.x = detector->H2_X + 1.0; 
    h2_goal->position.y = detector->H2_Y; 
    h2_goal->orientation.w = 1.0;
    navigator.GoToPose(h2_goal);
    while (rclcpp::ok() && !navigator.IsTaskComplete()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
    
    detector->resetCounters(); // Reset AFTER arrival
    navigator.Spin();
    while (rclcpp::ok() && !navigator.IsTaskComplete()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
    
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
        
        // generate coverage/search path
        auto waypoints = detector->generateCoveragePath();
        int wp_count = 0;

        // iterate over waypoints
        for (const auto& wp : waypoints) {
            // if we have found all humans, break
            if (verified_locations.size() >= static_cast<size_t>(missing_count)) break;

            // check if detector found a new candidate (dynamic obstacle)
            Candidate cand = detector->getBestNewCandidate();
            
            if (cand.x != 0.0) {
                bool already_checked = false;
                for(auto loc : verified_locations) {
                    if(std::hypot(cand.x - loc.x, cand.y - loc.y) < 2.0) already_checked = true;
                }

                if (!already_checked) {
                    std::cout << "\n[Main] INTERRUPT: Investigating candidate at (" << cand.x << ", " << cand.y << ")..." << std::endl;
                    // Cancel search path to investigate candidate
                    navigator.CancelTask();
                    
                    // create goal pose for candidate
                    auto cand_goal = std::make_shared<geometry_msgs::msg::Pose>();
                    cand_goal->position.x = cand.x; 
                    cand_goal->position.y = cand.y; 
                    cand_goal->orientation.w = 1.0;

                    // Go to candidate
                    navigator.GoToPose(cand_goal);
                    // verify
                    while (rclcpp::ok() && !navigator.IsTaskComplete()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
                    navigator.Spin(); 
                    while (rclcpp::ok() && !navigator.IsTaskComplete()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }

                    Candidate confirmed = detector->getBestNewCandidate();
                    
                    if (confirmed.x != 0.0 && std::hypot(confirmed.x - cand.x, confirmed.y - cand.y) < 1.0) {
                        std::cout << "[Main] CONFIRMED HUMAN FOUND!" << std::endl;
                        verified_locations.push_back(confirmed);
                    } else {
                        std::cout << "[Main] False positive (Ghost/Wall). Clearing and resuming..." << std::endl;
                        detector->clearCandidate(cand.x, cand.y);
                    }
                    continue; 
                }
            }

            wp_count++;
            if (std::hypot(wp.pose.position.x - 2.12, wp.pose.position.y - (-21.3)) < 2.0) continue;

            std::cout << "[Main] Navigating to WP " << wp_count << "/" << waypoints.size() << "..." << std::flush;
            auto goal = std::make_shared<geometry_msgs::msg::Pose>(wp.pose);
            // continue searching for humans at waypoints
            navigator.GoToPose(goal);

            // wait for waypoint to be reached
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

    // print final report
    detector->reportFindings(h1_found, h2_found, verified_locations);
    std::cout << "[Main] Mission Completed in " << minutes << " min " << seconds << " sec." << std::endl;

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}