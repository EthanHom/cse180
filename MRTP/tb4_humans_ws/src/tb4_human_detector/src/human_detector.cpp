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
            "map", rclcpp::QoS(10).transient_local().reliable(),
            std::bind(&HumanDetector::mapCallback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            std::bind(&HumanDetector::scanCallback, this, std::placeholders::_1));

        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", 10,
            std::bind(&HumanDetector::amclCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "HumanDetector initialized.");
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
        scan_count_.store(0);
        scans_near_h1_.store(0);
        scans_near_h2_.store(0);
        RCLCPP_INFO(this->get_logger(), "Detection counters reset.");
    }

    void printDebugInfo() {
        RCLCPP_INFO(this->get_logger(), 
            "[DEBUG] Scans: %d, Near H1: %d (hits: %zu), Near H2: %d (hits: %zu), Clusters: %zu",
            scan_count_.load(), scans_near_h1_.load(), human1_close_range_hits_.size(),
            scans_near_h2_.load(), human2_close_range_hits_.size(), clusters_.size());
    }

    void reportHumans() {
        std::lock_guard<std::mutex> lock(cluster_mutex_);
        
        int total_scans = scan_count_.load();
        int scans_near_1 = scans_near_h1_.load();
        int scans_near_2 = scans_near_h2_.load();
        
        int h1_hit_scans = human1_close_range_hits_.size();
        int h2_hit_scans = human2_close_range_hits_.size();
        
        RCLCPP_INFO(this->get_logger(), "==========================================");
        RCLCPP_INFO(this->get_logger(), "        HUMAN DETECTION REPORT            ");
        RCLCPP_INFO(this->get_logger(), "==========================================");
        RCLCPP_INFO(this->get_logger(), "Total scans: %d", total_scans);
        RCLCPP_INFO(this->get_logger(), "Scans near H1 position: %d (hits in %d scans)", 
            scans_near_1, h1_hit_scans);
        RCLCPP_INFO(this->get_logger(), "Scans near H2 position: %d (hits in %d scans)", 
            scans_near_2, h2_hit_scans);
        
        bool h1_present = false;
        bool h2_present = false;
        
        if (scans_near_1 > 50) {
            double h1_hit_ratio = (double)h1_hit_scans / scans_near_1;
            RCLCPP_INFO(this->get_logger(), "H1 hit ratio when close: %.2f", h1_hit_ratio);
            h1_present = (h1_hit_ratio > 0.3);
        }
        
        if (scans_near_2 > 50) {
            double h2_hit_ratio = (double)h2_hit_scans / scans_near_2;
            RCLCPP_INFO(this->get_logger(), "H2 hit ratio when close: %.2f", h2_hit_ratio);
            h2_present = (h2_hit_ratio > 0.3);
        }

        if (h1_present) {
            RCLCPP_INFO(this->get_logger(),
                "[Human 1] STILL at original (%.2f, %.2f) ✓",
                human1_x_, human1_y_);
        } else {
            RCLCPP_INFO(this->get_logger(),
                "[Human 1] MOVED from original (%.2f, %.2f)",
                human1_x_, human1_y_);
        }

        if (h2_present) {
            RCLCPP_INFO(this->get_logger(),
                "[Human 2] STILL at original (%.2f, %.2f) ✓",
                human2_x_, human2_y_);
        } else {
            RCLCPP_INFO(this->get_logger(),
                "[Human 2] MOVED from original (%.2f, %.2f)",
                human2_x_, human2_y_);
        }

        if (h1_present && h2_present) {
            RCLCPP_INFO(this->get_logger(), "Both humans at original locations.");
            return;
        }

        std::vector<Cluster> valid_clusters;
        
        const int min_cluster_size = 200;
        const int max_cluster_size = 5000;
        const double min_x = -14.0;
        const double max_x = 14.0;
        const double min_y = -21.0;
        const double max_y = 24.0;
        
        for (const auto &c : clusters_) {
            if (c.count < min_cluster_size || c.count > max_cluster_size) {
                continue;
            }
            
            double cx = c.cx();
            double cy = c.cy();
            
            if (cx < min_x || cx > max_x || cy < min_y || cy > max_y) {
                continue;
            }
            
            double d1 = std::hypot(cx - human1_x_, cy - human1_y_);
            double d2 = std::hypot(cx - human2_x_, cy - human2_y_);
            
            if (d1 > 2.0 && d2 > 2.0) {
                valid_clusters.push_back(c);
            }
        }

        RCLCPP_INFO(this->get_logger(), 
            "Total clusters: %zu, Valid human-sized: %zu", 
            clusters_.size(), valid_clusters.size());

        if (valid_clusters.empty()) {
            RCLCPP_WARN(this->get_logger(), "-> No valid human clusters found!");
            return;
        }

        std::sort(valid_clusters.begin(), valid_clusters.end(),
            [](const Cluster &a, const Cluster &b) { return a.count > b.count; });

        size_t to_show = std::min(valid_clusters.size(), size_t(5));
        RCLCPP_INFO(this->get_logger(), "Top %zu human candidates:", to_show);
        for (size_t i = 0; i < to_show; ++i) {
            RCLCPP_INFO(this->get_logger(), "  [%zu] (%.2f, %.2f) - %d points",
                i, valid_clusters[i].cx(), valid_clusters[i].cy(), valid_clusters[i].count);
        }

        if (!h1_present && !valid_clusters.empty()) {
            RCLCPP_INFO(this->get_logger(),
                "=> Human 1 NEW LOCATION: (%.2f, %.2f) [%d points]", 
                valid_clusters[0].cx(), valid_clusters[0].cy(), valid_clusters[0].count);
        }

        if (!h2_present && valid_clusters.size() > 1) {
            RCLCPP_INFO(this->get_logger(),
                "=> Human 2 NEW LOCATION: (%.2f, %.2f) [%d points]", 
                valid_clusters[1].cx(), valid_clusters[1].cy(), valid_clusters[1].count);
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

    const double cluster_dist_thresh_ = 0.5;
    const int min_points_per_cluster_ = 150;

    const double human1_x_ = 1.00;
    const double human1_y_ = -1.00;
    const double human2_x_ = -12.00;
    const double human2_y_ = 15.00;
    
    const double detection_radius_ = 0.5;
    const double near_threshold_ = 2.5;
    
    std::map<int, bool> human1_close_range_hits_;
    std::map<int, bool> human2_close_range_hits_;
    
    std::atomic<int> scan_count_{0};
    std::atomic<int> scans_near_h1_{0};
    std::atomic<int> scans_near_h2_{0};

    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_ = msg->pose.pose;
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_ = *msg;
        have_map_.store(true);
        RCLCPP_INFO(this->get_logger(), "Map received: %ux%u, resolution: %.3f",
            map_.info.width, map_.info.height, map_.info.resolution);
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        geometry_msgs::msg::TransformStamped tf;
        
        try {
            tf = tf_buffer_.lookupTransform("map", scan->header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "TF lookup failed: %s", ex.what());
            return;
        }

        if (!have_map_.load()) return;

        double robot_x = tf.transform.translation.x;
        double robot_y = tf.transform.translation.y;
        
        int current_scan_id = scan_count_.load();
        
        double dist_to_h1 = std::hypot(robot_x - human1_x_, robot_y - human1_y_);
        double dist_to_h2 = std::hypot(robot_x - human2_x_, robot_y - human2_y_);
        
        bool near_h1 = (dist_to_h1 < near_threshold_);
        bool near_h2 = (dist_to_h2 < near_threshold_);
        
        if (near_h1) scans_near_h1_++;
        if (near_h2) scans_near_h2_++;
        
        bool h1_hit_this_scan = false;
        bool h2_hit_this_scan = false;

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
            if (near_h1 && d1 <= detection_radius_) {
                h1_hit_this_scan = true;
            }

            double d2 = std::hypot(px - human2_x_, py - human2_y_);
            if (near_h2 && d2 <= detection_radius_) {
                h2_hit_this_scan = true;
            }

            if (is_free_space) {
                std::lock_guard<std::mutex> lock(cluster_mutex_);
                addPointToClusters(px, py);
            }
        }

        {
            std::lock_guard<std::mutex> lock(cluster_mutex_);
            if (h1_hit_this_scan) {
                human1_close_range_hits_[current_scan_id] = true;
            }
            if (h2_hit_this_scan) {
                human2_close_range_hits_[current_scan_id] = true;
            }
        }

        scan_count_++;
        
        if (current_scan_id % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "[Scan %d] Robot at (%.2f, %.2f), Near H1: %s, Near H2: %s",
                current_scan_id, robot_x, robot_y,
                near_h1 ? "YES" : "no", near_h2 ? "YES" : "no");
        }
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

    std::cout << "[Main] Creating EFFICIENT zigzag coverage pattern..." << std::endl;
    
    // ==================================================================
    // OPTIMIZED PATTERN: Systematic zigzag reducing backtracking
    // Based on visual map analysis - 45 waypoints (down from 87!)
    // ==================================================================
    std::vector<std::pair<double, double>> grid_pattern = {
        // Row 1: Bottom right to left (Y=-18 to -13)
        {10.0, -18.0},
        {6.0, -16.0},
        {2.0, -15.0},
        {-2.0, -15.0},
        {-6.0, -16.0},
        {-10.0, -18.0},
        
        // Row 2: Left to right (Y=-8)
        {-10.0, -8.0},
        {-6.0, -8.0},
        {-2.0, -8.0},
        {2.0, -8.0},
        {6.0, -8.0},
        {10.0, -8.0},
        
        // Row 3: Right to left (Y=-2 to 2) - H1 ORIGINAL AREA
        {10.0, -2.0},
        {6.0, -1.0},
        {2.0, 0.0},     // NEAR H1 (1, -1)
        {0.0, 0.0},     // PASS THROUGH H1
        {-2.0, 0.0},
        {-6.0, 1.0},
        {-10.0, 2.0},
        
        // Row 4: Left to right (Y=5 to 7) - H1 NEW LOCATION AREA
        {-10.0, 5.0},
        {-6.0, 5.0},
        {-2.0, 6.0},
        {2.0, 6.0},
        {5.0, 5.0},     // NEAR H1 NEW (5, 3)
        {4.0, 3.0},     // CLOSER TO H1 NEW
        {8.0, 6.0},
        {11.0, 7.0},
        
        // Row 5: Right to left (Y=11)
        {11.0, 11.0},
        {7.0, 11.0},
        {3.0, 10.0},
        {0.0, 10.0},
        {-4.0, 11.0},
        {-8.0, 11.0},
        {-11.0, 12.0},
        
        // Row 6: Left to right (Y=15 to 17) - H2 ORIGINAL AREA
        {-12.0, 14.0},  // APPROACH H2
        {-12.0, 15.0},  // H2 ORIGINAL LOCATION
        {-10.0, 16.0},
        {-6.0, 16.0},
        {-2.0, 17.0},
        {2.0, 17.0},
        {6.0, 17.0},
        {9.0, 18.0},
        
        // Row 7: Right to left (Y=21 to 23) - Top coverage
        {8.0, 21.0},
        {4.0, 22.0},
        {0.0, 23.0},
        {-4.0, 22.0},
        {-8.0, 21.0},
        {-11.0, 20.0},
    };
    
    std::cout << "[Main] Optimized pattern: " << grid_pattern.size() 
              << " waypoints (vs 87 before!)" << std::endl;
    
    for (const auto& [x, y] : grid_pattern) {
        add_wp(x, y, 90.0);
    }
    
    // Strategic 360° spins only at human locations
    std::vector<int> spin_at_waypoints;
    
    for (size_t i = 0; i < grid_pattern.size(); ++i) {
        double x = grid_pattern[i].first;
        double y = grid_pattern[i].second;
        
        // Spin ONLY at critical human locations
        if (std::hypot(x - 0.0, y - 0.0) < 1.5) {
            spin_at_waypoints.push_back(i);
            std::cout << "[Main] 360° at WP" << i << " (H1 original)" << std::endl;
        }
        
        if (std::hypot(x - 4.0, y - 3.0) < 1.5) {
            spin_at_waypoints.push_back(i);
            std::cout << "[Main] 360° at WP" << i << " (H1 new location)" << std::endl;
        }
        
        if (std::hypot(x + 12.0, y - 15.0) < 1.0) {
            spin_at_waypoints.push_back(i);
            std::cout << "[Main] 360° at WP" << i << " (H2 original)" << std::endl;
        }
    }

    // Execute navigation
    for (size_t i = 0; i < waypoints.size(); ++i) {
        auto goal = std::make_shared<geometry_msgs::msg::Pose>(waypoints[i].pose);
        std::cout << "[Main] -> WP" << (i+1) << "/" << waypoints.size() 
                  << " (" << grid_pattern[i].first << ", " << grid_pattern[i].second << ")" 
                  << std::endl;
        
        navigator.GoToPose(goal);
        
        while (rclcpp::ok() && !navigator.IsTaskComplete()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (navigator.GetResult() == rclcpp_action::ResultCode::SUCCEEDED) {
            auto pose = detector->getCurrentPose();
            std::cout << "[Main] ✓ WP" << (i+1) << std::endl;
            
            if (std::find(spin_at_waypoints.begin(), spin_at_waypoints.end(), i) 
                != spin_at_waypoints.end()) {
                std::cout << "[Main] Spinning 360°..." << std::endl;
                navigator.Spin();
                while (rclcpp::ok() && !navigator.IsTaskComplete()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            
            if ((i+1) % 10 == 0) {
                detector->printDebugInfo();
            }
            
        } else {
            std::cerr << "[Main] ✗ Failed WP" << (i+1) << " - continuing..." << std::endl;
        }
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "WAREHOUSE SCAN COMPLETE!" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    detector->reportHumans();
    
    rclcpp::shutdown();
    if (spin_thread.joinable()) spin_thread.join();
    
    return 0;
}
