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

        RCLCPP_INFO(this->get_logger(), "------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Analyzing detections (filtering static obstacles)...");

        struct ScoredCandidate {
            double x, y;
            int hit_scans, total_scans;
            double score;
        };
        
        std::vector<ScoredCandidate> candidates;
        
        for (const auto& [key, detection] : dynamic_detections_) {
            if (detection.total_scans < 10) continue;
            if (detection.hit_scans < 3) continue;
            
            double ratio = detection.hit_ratio();
            if (ratio < 0.05) continue;
            
            // FILTER 1: Away from original human positions
            double d1 = std::hypot(detection.x - human1_x_, detection.y - human1_y_);
            double d2 = std::hypot(detection.x - human2_x_, detection.y - human2_y_);
            if (d1 < 2.0 || d2 < 2.0) continue;
            
            // FILTER 2: NOT a known static obstacle
            if (isKnownStaticObstacle(detection.x, detection.y)) {
                continue;
            }
            
            // FILTER 3: Within warehouse bounds (-14 to 14, -24 to 24)
            if (detection.x < -13.5 || detection.x > 13.5) continue;
            if (detection.y < -23.5 || detection.y > 23.5) continue;
            
            // FILTER 4: Away from warehouse edges (1.5m safety margin)
            if (detection.x < -12.0 || detection.x > 12.0) continue;
            if (detection.y < -22.0 || detection.y > 22.0) continue;
            
            // CALCULATE CONFIDENCE SCORE (0-100)
            double score = 0.0;
            
            // Factor 1: Hit ratio (max 30 points)
            score += ratio * 30.0;
            
            // Factor 2: Scan count (max 40 points, capped at 200 scans)
            score += std::min(detection.hit_scans / 5.0, 40.0);
            
            // Factor 3: Distance from edges (max 30 points)
            double edge_dist_x = std::min(detection.x + 12.0, 12.0 - detection.x);
            double edge_dist_y = std::min(detection.y + 22.0, 22.0 - detection.y);
            double min_edge_dist = std::min(edge_dist_x, edge_dist_y);
            score += std::min(min_edge_dist * 3.0, 30.0);
            
            candidates.push_back({detection.x, detection.y, 
                detection.hit_scans, detection.total_scans, score});
        }

        // Sort by score (highest first)
        std::sort(candidates.begin(), candidates.end(),
            [](const ScoredCandidate &a, const ScoredCandidate &b) { 
                return a.score > b.score; 
            });

        RCLCPP_INFO(this->get_logger(), "Found %zu candidates (before clustering)", candidates.size());

        // Show top candidates
        size_t to_show = std::min(candidates.size(), size_t(10));
        if (to_show > 0) {
            RCLCPP_INFO(this->get_logger(), "Top %zu candidates by score:", to_show);
            for (size_t i = 0; i < to_show; ++i) {
                RCLCPP_INFO(this->get_logger(), 
                    "  [%zu] (%.2f, %.2f) - %d/%d scans, score: %.1f",
                    i, candidates[i].x, candidates[i].y,
                    candidates[i].hit_scans, candidates[i].total_scans,
                    candidates[i].score);
            }
        }

        // Cluster nearby candidates with weighted averaging
        std::vector<ScoredCandidate> clustered;
        for (const auto& cand : candidates) {
            bool merged = false;
            for (auto& existing : clustered) {
                if (std::hypot(cand.x - existing.x, cand.y - existing.y) < 1.5) {
                    double w1 = existing.score;
                    double w2 = cand.score;
                    double total_w = w1 + w2;
                    existing.x = (existing.x * w1 + cand.x * w2) / total_w;
                    existing.y = (existing.y * w1 + cand.y * w2) / total_w;
                    existing.hit_scans += cand.hit_scans;
                    existing.total_scans += cand.total_scans;
                    existing.score = std::max(existing.score, cand.score);
                    merged = true;
                    break;
                }
            }
            if (!merged) {
                clustered.push_back(cand);
            }
        }

        // Re-sort after clustering
        std::sort(clustered.begin(), clustered.end(),
            [](const ScoredCandidate &a, const ScoredCandidate &b) { 
                return a.score > b.score; 
            });

        RCLCPP_INFO(this->get_logger(), "After clustering: %zu unique locations", clustered.size());

        // Report new locations
        if (!h1_present) {
            if (!clustered.empty()) {
                RCLCPP_INFO(this->get_logger(), 
                    "=> Human 1 NEW LOCATION: (%.2f, %.2f) ✓ [score: %.1f, %d scans]",
                    clustered[0].x, clustered[0].y, clustered[0].score, clustered[0].hit_scans);
            } else {
                RCLCPP_WARN(this->get_logger(), "=> Human 1 new location: NOT FOUND");
            }
        }

        if (!h2_present) {
            size_t idx = h1_present ? 0 : 1;
            if (idx < clustered.size()) {
                RCLCPP_INFO(this->get_logger(), 
                    "=> Human 2 NEW LOCATION: (%.2f, %.2f) ✓ [score: %.1f, %d scans]",
                    clustered[idx].x, clustered[idx].y, clustered[idx].score, clustered[idx].hit_scans);
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
        const double threshold = 2.0;
        for (const auto& obstacle : known_obstacles_) {
            double dist = std::hypot(x - obstacle.first, y - obstacle.second);
            if (dist < threshold) {
                return true;
            }
        }
        return false;
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

    std::cout << "[Main] Creating FULL warehouse coverage..." << std::endl;
    std::cout << "[Main] Warehouse corners: (-14,±24), (14,±24)" << std::endl;
    std::cout << "[Main] Coverage zone (2m margin): X(-12 to 12), Y(-22 to 22)" << std::endl;
    
    // ================================================================
    // SYSTEMATIC ZIGZAG GRID COVERAGE
    // - Warehouse: 28m × 48m (-14 to 14, -24 to 24)
    // - Coverage: 24m × 44m (-12 to 12, -22 to 22) with 2m margin
    // - LiDAR range: 3m
    // - Waypoint spacing: 4m (ensures 1m overlap)
    // - Pattern: 7 columns × 12 rows = 84 waypoints
    // ================================================================
    std::vector<std::pair<double, double>> grid_pattern;
    
    // Y range: -22 to 22 in 4m steps = 12 rows
    // X range: -12 to 12 in 4m steps = 7 columns (-12, -8, -4, 0, 4, 8, 12)
    
    for (double y = -22.0; y <= 22.0; y += 4.0) {
        int row_num = static_cast<int>((y + 22.0) / 4.0);
        if (row_num % 2 == 0) {  // Even row: left to right
            for (double x = -12.0; x <= 12.0; x += 4.0) {
                grid_pattern.push_back({x, y});
            }
        } else {  // Odd row: right to left
            for (double x = 12.0; x >= -12.0; x -= 4.0) {
                grid_pattern.push_back({x, y});
            }
        }
    }
    
    std::cout << "[Main] Generated " << grid_pattern.size() << " waypoints" << std::endl;
    std::cout << "[Main] Pattern: 7 columns (-12 to 12) × 12 rows (-22 to 22)" << std::endl;
    std::cout << "[Main] Estimated time: " << (grid_pattern.size() * 8 / 60) << "-" 
              << (grid_pattern.size() * 12 / 60) << " minutes" << std::endl;
    
    // Verify coverage of critical locations
    std::vector<std::pair<double, double>> test_locations = {
        {1.0, -1.0}, {-12.0, 15.0},      // Original H1, H2
        {5.0, 3.0}, {10.0, 12.0},        // Test locations
        {-4.0, -10.0}, {12.0, 11.0}      // Edge locations
    };
    
    for (const auto& [tx, ty] : test_locations) {
        bool covered = false;
        for (const auto& [x, y] : grid_pattern) {
            if (std::hypot(x - tx, y - ty) <= 3.0) {  // Within LiDAR range
                covered = true;
                std::cout << "[Main] ✓ Location (" << tx << ", " << ty 
                          << ") covered by waypoint (" << x << ", " << y << ")" << std::endl;
                break;
            }
        }
        if (!covered) {
            std::cerr << "[Main] ⚠ WARNING: Location (" << tx << ", " << ty 
                      << ") NOT COVERED!" << std::endl;
        }
    }
    
    for (const auto& [x, y] : grid_pattern) {
        add_wp(x, y, 90.0);
    }
    
    // 360° spins at strategic locations
    std::vector<int> spin_at_waypoints;
    
    for (size_t i = 0; i < grid_pattern.size(); ++i) {
        double x = grid_pattern[i].first;
        double y = grid_pattern[i].second;
        
        // Spin at H1 original (1, -1)
        if (std::hypot(x - 1.0, y + 1.0) <= 3.0) {
            spin_at_waypoints.push_back(i);
            std::cout << "[Main] 360° at WP" << (i+1) << " near H1 original (1,-1)" << std::endl;
        }
        
        // Spin at H2 original (-12, 15)
        if (std::hypot(x + 12.0, y - 15.0) <= 3.0) {
            spin_at_waypoints.push_back(i);
            std::cout << "[Main] 360° at WP" << (i+1) << " near H2 original (-12,15)" << std::endl;
        }
        
        // Spin near test location (5, 3)
        if (std::hypot(x - 5.0, y - 3.0) <= 3.0) {
            spin_at_waypoints.push_back(i);
            std::cout << "[Main] 360° at WP" << (i+1) << " near (5,3)" << std::endl;
        }
        
        // Spin near edge location (12, 11)
        if (std::hypot(x - 12.0, y - 11.0) <= 3.0) {
            spin_at_waypoints.push_back(i);
            std::cout << "[Main] 360° at WP" << (i+1) << " near edge (12,11)" << std::endl;
        }
    }
    
    std::cout << "[Main] 360° spins at " << spin_at_waypoints.size() << " locations" << std::endl;

    // Execute navigation with progress tracking
    int success_count = 0;
    int fail_count = 0;
    auto start_time = std::chrono::steady_clock::now();
    
    for (size_t i = 0; i < waypoints.size(); ++i) {
        auto goal = std::make_shared<geometry_msgs::msg::Pose>(waypoints[i].pose);
        
        // Progress indicator every 10 waypoints
        if (i % 10 == 0 || i == waypoints.size() - 1) {
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            int mins = std::chrono::duration_cast<std::chrono::minutes>(elapsed).count();
            std::cout << "\n[Progress] " << (i+1) << "/" << waypoints.size() 
                      << " (" << (100*(i+1)/waypoints.size()) << "%) - " 
                      << mins << " min elapsed" << std::endl;
        }
        
        std::cout << "[Main] -> WP" << (i+1) << "/" << waypoints.size() 
                  << " (" << grid_pattern[i].first << ", " << grid_pattern[i].second << ")" 
                  << std::endl;
        
        navigator.GoToPose(goal);
        
        while (rclcpp::ok() && !navigator.IsTaskComplete()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (navigator.GetResult() == rclcpp_action::ResultCode::SUCCEEDED) {
            std::cout << "[Main] ✓ WP" << (i+1) << std::endl;
            success_count++;
            
            // Perform 360° spin at strategic locations
            if (std::find(spin_at_waypoints.begin(), spin_at_waypoints.end(), i) 
                != spin_at_waypoints.end()) {
                std::cout << "[Main] Spinning 360°..." << std::endl;
                navigator.Spin();
                while (rclcpp::ok() && !navigator.IsTaskComplete()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            
            // Print debug info periodically
            if ((i+1) % 20 == 0) {
                detector->printDebugInfo();
            }
            
        } else {
            std::cerr << "[Main] ✗ Failed WP" << (i+1) << " - continuing..." << std::endl;
            fail_count++;
            
            // Safety check: abort if too many failures
            if (fail_count > waypoints.size() * 0.3) {
                std::cerr << "[Main] ⚠ Too many failures (" << fail_count 
                          << "/" << waypoints.size() << "), aborting scan..." << std::endl;
                break;
            }
        }
    }

    auto total_time = std::chrono::steady_clock::now() - start_time;
    int total_mins = std::chrono::duration_cast<std::chrono::minutes>(total_time).count();

    std::cout << "\n========================================" << std::endl;
    std::cout << "     FULL WAREHOUSE SCAN COMPLETE!     " << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Waypoints: " << success_count << "/" << waypoints.size() 
              << " successful (" << (100*success_count/waypoints.size()) << "%)" << std::endl;
    std::cout << "Failures: " << fail_count << std::endl;
    std::cout << "Time: " << total_mins << " minutes" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    detector->reportHumans();
    
    rclcpp::shutdown();
    if (spin_thread.joinable()) spin_thread.join();
    
    return 0;
}
