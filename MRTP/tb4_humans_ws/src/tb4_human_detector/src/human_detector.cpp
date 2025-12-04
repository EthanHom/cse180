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
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <map>

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

struct DetectionPoint {
    double x, y;
    int count{0};
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

    void resetDetection() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        all_obstacles_.clear();
        h1_detections_.store(0);
        h2_detections_.store(0);
        scans_near_h1_.store(0);
        scans_near_h2_.store(0);
        RCLCPP_INFO(this->get_logger(), "Detection counters reset.");
    }

    void reportHumans() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        int scans_h1 = scans_near_h1_.load();
        int scans_h2 = scans_near_h2_.load();
        int hits_h1 = h1_detections_.load();
        int hits_h2 = h2_detections_.load();
        
        RCLCPP_INFO(this->get_logger(), "==========================================");
        RCLCPP_INFO(this->get_logger(), "        HUMAN DETECTION REPORT            ");
        RCLCPP_INFO(this->get_logger(), "==========================================");
        RCLCPP_INFO(this->get_logger(), "H1 original (%.2f, %.2f): %d/%d scans (%.1f%%)", 
            human1_x_, human1_y_, hits_h1, scans_h1, 
            scans_h1 > 0 ? 100.0*hits_h1/scans_h1 : 0.0);
        RCLCPP_INFO(this->get_logger(), "H2 original (%.2f, %.2f): %d/%d scans (%.1f%%)", 
            human2_x_, human2_y_, hits_h2, scans_h2,
            scans_h2 > 0 ? 100.0*hits_h2/scans_h2 : 0.0);
        
        bool h1_present = false;
        bool h2_present = false;
        
        if (scans_h1 > 15) {
            double ratio = (double)hits_h1 / scans_h1;
            h1_present = (ratio > 0.08);
        }
        
        if (scans_h2 > 15) {
            double ratio = (double)hits_h2 / scans_h2;
            h2_present = (ratio > 0.08);
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
            RCLCPP_INFO(this->get_logger(), "==========================================");
            RCLCPP_INFO(this->get_logger(), "Both humans remain at original locations.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Analyzing obstacle detections for moved humans...");

        std::vector<DetectionPoint> candidates;
        
        for (const auto& [loc, count] : all_obstacles_) {
            if (count < 8) continue;
            
            double px = loc.first / 10.0;
            double py = loc.second / 10.0;
            
            double d1 = std::hypot(px - human1_x_, py - human1_y_);
            double d2 = std::hypot(px - human2_x_, py - human2_y_);
            if (d1 < 1.5 || d2 < 1.5) continue;
            
            if (isKnownStaticObstacle(px, py)) continue;
            
            if (px < -11.5 || px > 11.5 || py < -21.5 || py > 21.5) continue;
            
            candidates.push_back({px, py, count});
        }

        std::sort(candidates.begin(), candidates.end(),
            [](const DetectionPoint &a, const DetectionPoint &b) { 
                return a.count > b.count; 
            });

        std::vector<DetectionPoint> clustered;
        for (const auto& cand : candidates) {
            bool merged = false;
            for (auto& existing : clustered) {
                if (std::hypot(cand.x - existing.x, cand.y - existing.y) < 1.0) {
                    double w1 = existing.count;
                    double w2 = cand.count;
                    existing.x = (existing.x * w1 + cand.x * w2) / (w1 + w2);
                    existing.y = (existing.y * w1 + cand.y * w2) / (w1 + w2);
                    existing.count += cand.count;
                    merged = true;
                    break;
                }
            }
            if (!merged) {
                clustered.push_back(cand);
            }
        }

        std::sort(clustered.begin(), clustered.end(),
            [](const DetectionPoint &a, const DetectionPoint &b) { 
                return a.count > b.count; 
            });

        RCLCPP_INFO(this->get_logger(), "Found %zu potential human locations", clustered.size());

        if (!h1_present && !clustered.empty()) {
            RCLCPP_INFO(this->get_logger(), 
                "=> Human 1 NEW LOCATION: (%.2f, %.2f) [%d detections]",
                clustered[0].x, clustered[0].y, clustered[0].count);
        }

        if (!h2_present) {
            size_t idx = h1_present ? 0 : 1;
            if (idx < clustered.size()) {
                RCLCPP_INFO(this->get_logger(), 
                    "=> Human 2 NEW LOCATION: (%.2f, %.2f) [%d detections]",
                    clustered[idx].x, clustered[idx].y, clustered[idx].count);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "==========================================");
    }

private:
    std::mutex data_mutex_;
    std::mutex pose_mutex_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

    nav_msgs::msg::OccupancyGrid map_;
    std::atomic<bool> have_map_{false};
    geometry_msgs::msg::Pose current_pose_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::map<std::pair<int,int>, int> all_obstacles_;
    std::vector<std::pair<double, double>> known_obstacles_;

    const double human1_x_ = 1.00;
    const double human1_y_ = -1.00;
    const double human2_x_ = -12.00;
    const double human2_y_ = 15.00;
    
    std::atomic<int> h1_detections_{0};
    std::atomic<int> h2_detections_{0};
    std::atomic<int> scans_near_h1_{0};
    std::atomic<int> scans_near_h2_{0};

    void initializeKnownObstacles() {
        known_obstacles_ = {
            {-8.5, -13.0}, {6.5, -13.0}, {-1.5, -13.0},
            {13.5, 4.5}, {10.0, 4.5}, {13.5, -21.0}, {13.5, -15.0},
            {0.4, -2.0}, {3.5, 9.5}, {-1.3, 18.5},
            {-10.0, 21.5}, {-7.0, 23.6}, {-4.0, 21.5},
            {-10.4, 14.75}, {-10.4, 10.5}, {-10.4, 6.5}, {-12.85, 4.85},
            {14.3, -5.5}, {14.3, -4.0},
            {-11.5, 6.4}, {-14.0, 6.5}, {-12.7, 6.5},
        };
    }

    bool isKnownStaticObstacle(double x, double y) const {
        for (const auto& [ox, oy] : known_obstacles_) {
            if (std::hypot(x - ox, y - oy) < 1.5) return true;
        }
        return false;
    }

    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_ = msg->pose.pose;
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        map_ = *msg;
        have_map_.store(true);
        RCLCPP_INFO(this->get_logger(), "Map received: %ux%u @ %.3fm/px", 
            map_.info.width, map_.info.height, map_.info.resolution);
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
        
        bool near_h1 = std::hypot(robot_x - human1_x_, robot_y - human1_y_) < 2.5;
        bool near_h2 = std::hypot(robot_x - human2_x_, robot_y - human2_y_) < 2.5;
        
        if (near_h1) scans_near_h1_++;
        if (near_h2) scans_near_h2_++;
        
        bool h1_detected = false;
        bool h2_detected = false;

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float r = scan->ranges[i];
            if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) continue;

            float angle = scan->angle_min + i * scan->angle_increment;

            geometry_msgs::msg::PointStamped p_laser, p_map;
            p_laser.header = scan->header;
            p_laser.point.x = r * std::cos(angle);
            p_laser.point.y = r * std::sin(angle);
            p_laser.point.z = 0.0;

            tf2::doTransform(p_laser, p_map, tf);
            
            double px = p_map.point.x;
            double py = p_map.point.y;

            if (near_h1 && std::hypot(px - human1_x_, py - human1_y_) < 0.8) {
                h1_detected = true;
            }
            if (near_h2 && std::hypot(px - human2_x_, py - human2_y_) < 0.8) {
                h2_detected = true;
            }

            if (r < 3.0) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                int gx = static_cast<int>(std::round(px * 10.0));
                int gy = static_cast<int>(std::round(py * 10.0));
                all_obstacles_[{gx, gy}]++;
            }
        }

        if (h1_detected) h1_detections_++;
        if (h2_detected) h2_detections_++;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    auto detector = std::make_shared<HumanDetector>();
    Navigator navigator(true);

    std::thread spin_thread([&]() { rclcpp::spin(detector); });

    std::cout << "\n[Main] Waiting for map..." << std::endl;
    while (rclcpp::ok() && !detector->hasMap()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "[Main] Map received!\n" << std::endl;
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
    std::vector<std::string> waypoint_notes;
    
    auto add_wp = [&](double x, double y, double yaw_deg, const std::string& note = "") {
        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = "map";
        p.pose.position.x = x;
        p.pose.position.y = y;
        double rad = yaw_deg * M_PI / 180.0;
        p.pose.orientation.z = std::sin(rad / 2.0);
        p.pose.orientation.w = std::cos(rad / 2.0);
        waypoints.push_back(p);
        waypoint_notes.push_back(note);
    };

    std::cout << "========================================" << std::endl;
    std::cout << "   MAP-OPTIMIZED COVERAGE WAYPOINTS" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Map: 1006×1674 @ 0.03m/px" << std::endl;
    std::cout << "Area: 30m × 50m warehouse" << std::endl;
    std::cout << "Strategy: Corridor-following coverage" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // ================================================================
    // OPTIMAL WAYPOINTS - DESIGNED FROM ACTUAL MAP
    // Based on navigable corridors visible in my_warehouse_map.jpg
    // ================================================================
    
    std::cout << "Generating optimal waypoints:\n" << std::endl;
    
    // SOUTH SECTION - Between 3 vertical shelves
    add_wp(7.0, -17.0, 90.0, "South-East corridor");
    add_wp(3.0, -15.0, 90.0, "East-Center shelf corridor");
    add_wp(-2.0, -15.0, 90.0, "Center-West shelf corridor");
    add_wp(-8.0, -17.0, 90.0, "South-West corridor");
    
    // CENTER AREA - H1 vicinity and open space
    add_wp(-5.0, -5.0, 90.0, "West-Center area");
    add_wp(1.0, -1.0, 90.0, "★ H1 ORIGINAL (1, -1)");
    add_wp(5.0, -5.0, 90.0, "East-Center area");
    add_wp(0.0, 5.0, 90.0, "Center open area");
    
    // NORTH SECTION - Above horizontal shelves
    add_wp(8.0, 12.0, 90.0, "North-East area");
    add_wp(-10.0, 15.0, 90.0, "West corridor (H2 vicinity)");
    add_wp(-12.0, 18.0, 90.0, "★ H2 ORIGINAL (-12, 15)");
    add_wp(0.0, 18.0, 90.0, "North-Center");
    
    // EDGE CHECKS - For humans moved to corners
    add_wp(10.0, 5.0, 90.0, "East edge mid");
    add_wp(10.0, -10.0, 90.0, "East edge south");
    
    std::cout << "[Main] Generated " << waypoints.size() << " waypoints\n" << std::endl;
    
    // Print waypoint plan
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "  WP" << (i+1) << ": (" 
                  << waypoints[i].pose.position.x << ", " 
                  << waypoints[i].pose.position.y << ") - "
                  << waypoint_notes[i] << std::endl;
    }
    
    // Identify spin locations (H1 and H2)
    std::vector<int> spin_at;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        double x = waypoints[i].pose.position.x;
        double y = waypoints[i].pose.position.y;
        if ((std::hypot(x - 1.0, y + 1.0) < 1.0) ||  // H1
            (std::hypot(x + 12.0, y - 15.0) < 4.0)) { // H2
            spin_at.push_back(i);
        }
    }
    
    std::cout << "\n[Main] Will perform 360° spin at " << spin_at.size() 
              << " strategic locations (H1 & H2 originals)\n" << std::endl;
    std::cout << "Estimated completion time: 10-15 minutes\n" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // Execute navigation
    int success = 0, failed = 0;
    auto start = std::chrono::steady_clock::now();
    
    for (size_t i = 0; i < waypoints.size(); ++i) {
        auto goal = std::make_shared<geometry_msgs::msg::Pose>(waypoints[i].pose);
        
        auto elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start).count();
        int mins = elapsed_sec / 60;
        int secs = elapsed_sec % 60;
        
        std::cout << "[WP " << (i+1) << "/" << waypoints.size() << "] ";
        std::cout << waypoint_notes[i];
        std::cout << " | " << mins << ":" << (secs < 10 ? "0" : "") << secs << " elapsed" << std::endl;
        
        navigator.GoToPose(goal);
        
        while (rclcpp::ok() && !navigator.IsTaskComplete()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (navigator.GetResult() == rclcpp_action::ResultCode::SUCCEEDED) {
            std::cout << "  ✓ Reached successfully" << std::endl;
            success++;
            
            // 360° spin at H1/H2 locations
            if (std::find(spin_at.begin(), spin_at.end(), i) != spin_at.end()) {
                std::cout << "  → Performing 360° scan..." << std::endl;
                navigator.Spin();
                while (rclcpp::ok() && !navigator.IsTaskComplete()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        } else {
            std::cout << "  ✗ Navigation failed" << std::endl;
            failed++;
            if (failed > 3) {
                std::cerr << "\n[ERROR] Too many failures (" << failed << "), aborting mission." << std::endl;
                break;
            }
        }
    }

    auto runtime = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - start).count();
    int runtime_min = runtime / 60;
    int runtime_sec = runtime % 60;

    std::cout << "\n========================================" << std::endl;
    std::cout << "     WAREHOUSE COVERAGE COMPLETE" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Waypoints completed: " << success << "/" << waypoints.size() 
              << " (" << (waypoints.size() > 0 ? (100*success/waypoints.size()) : 0) << "%)" << std::endl;
    std::cout << "Failed attempts: " << failed << std::endl;
    std::cout << "Total runtime: " << runtime_min << "m " << runtime_sec << "s" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    detector->reportHumans();
    
    rclcpp::shutdown();
    if (spin_thread.joinable()) spin_thread.join();
    
    return 0;
}
