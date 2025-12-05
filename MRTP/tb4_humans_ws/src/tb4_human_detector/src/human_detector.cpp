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

// #include <memory>
// #include <vector>
// #include <cmath>
// #include <iostream>
// #include <chrono>
// #include <thread>
// #include <mutex>
// #include <atomic>
// #include <map>

// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"
// #include "geometry_msgs/msg/pose.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
// #include "geometry_msgs/msg/point_stamped.hpp"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include "navigation/navigation.hpp"

// struct DetectionPoint {
//     double x, y;
//     int count{0};
// };

// class HumanDetector : public rclcpp::Node {
// public:
//     HumanDetector()
//         : Node("tb4_human_detector"),
//           tf_buffer_(this->get_clock()),
//           tf_listener_(tf_buffer_)
//     {
//         if (!this->has_parameter("use_sim_time")) {
//             this->declare_parameter("use_sim_time", true);
//         }
//         this->set_parameter(rclcpp::Parameter("use_sim_time", true));

//         initializeKnownObstacles();

//         map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//             "map", rclcpp::QoS(10).transient_local().reliable(),
//             std::bind(&HumanDetector::mapCallback, this, std::placeholders::_1));

//         scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//             "scan", rclcpp::SensorDataQoS(),
//             std::bind(&HumanDetector::scanCallback, this, std::placeholders::_1));

//         amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
//             "amcl_pose", 10,
//             std::bind(&HumanDetector::amclCallback, this, std::placeholders::_1));

//         RCLCPP_INFO(this->get_logger(), "HumanDetector initialized with %zu known obstacles.", 
//             known_obstacles_.size());
//     }

//     bool hasMap() const { return have_map_.load(); }

//     void resetDetection() {
//         std::lock_guard<std::mutex> lock(data_mutex_);
//         all_obstacles_.clear();
//         h1_detections_.store(0);
//         h2_detections_.store(0);
//         scans_near_h1_.store(0);
//         scans_near_h2_.store(0);
//         RCLCPP_INFO(this->get_logger(), "Detection counters reset.");
//     }

//     void reportHumans() {
//         std::lock_guard<std::mutex> lock(data_mutex_);
        
//         int scans_h1 = scans_near_h1_.load();
//         int scans_h2 = scans_near_h2_.load();
//         int hits_h1 = h1_detections_.load();
//         int hits_h2 = h2_detections_.load();
        
//         RCLCPP_INFO(this->get_logger(), "==========================================");
//         RCLCPP_INFO(this->get_logger(), "        HUMAN DETECTION REPORT            ");
//         RCLCPP_INFO(this->get_logger(), "==========================================");
//         RCLCPP_INFO(this->get_logger(), "H1 original (%.2f, %.2f): %d/%d scans (%.1f%%)", 
//             human1_x_, human1_y_, hits_h1, scans_h1, 
//             scans_h1 > 0 ? 100.0*hits_h1/scans_h1 : 0.0);
//         RCLCPP_INFO(this->get_logger(), "H2 original (%.2f, %.2f): %d/%d scans (%.1f%%)", 
//             human2_x_, human2_y_, hits_h2, scans_h2,
//             scans_h2 > 0 ? 100.0*hits_h2/scans_h2 : 0.0);
        
//         bool h1_present = false;
//         bool h2_present = false;
        
//         if (scans_h1 > 15) {
//             double ratio = (double)hits_h1 / scans_h1;
//             h1_present = (ratio > 0.08);
//         }
        
//         if (scans_h2 > 15) {
//             double ratio = (double)hits_h2 / scans_h2;
//             h2_present = (ratio > 0.08);
//         }

//         if (h1_present) {
//             RCLCPP_INFO(this->get_logger(), "[Human 1] STILL at (%.2f, %.2f) ✓", human1_x_, human1_y_);
//         } else {
//             RCLCPP_INFO(this->get_logger(), "[Human 1] MOVED from (%.2f, %.2f)", human1_x_, human1_y_);
//         }

//         if (h2_present) {
//             RCLCPP_INFO(this->get_logger(), "[Human 2] STILL at (%.2f, %.2f) ✓", human2_x_, human2_y_);
//         } else {
//             RCLCPP_INFO(this->get_logger(), "[Human 2] MOVED from (%.2f, %.2f)", human2_x_, human2_y_);
//         }

//         if (h1_present && h2_present) {
//             RCLCPP_INFO(this->get_logger(), "==========================================");
//             RCLCPP_INFO(this->get_logger(), "Both humans remain at original locations.");
//             return;
//         }

//         RCLCPP_INFO(this->get_logger(), "------------------------------------------");
//         RCLCPP_INFO(this->get_logger(), "Analyzing obstacle detections for moved humans...");

//         std::vector<DetectionPoint> candidates;
        
//         for (const auto& [loc, count] : all_obstacles_) {
//             if (count < 8) continue;
            
//             double px = loc.first / 10.0;
//             double py = loc.second / 10.0;
            
//             double d1 = std::hypot(px - human1_x_, py - human1_y_);
//             double d2 = std::hypot(px - human2_x_, py - human2_y_);
//             if (d1 < 1.5 || d2 < 1.5) continue;
            
//             if (isKnownStaticObstacle(px, py)) continue;
            
//             if (px < -11.5 || px > 11.5 || py < -21.5 || py > 21.5) continue;
            
//             candidates.push_back({px, py, count});
//         }

//         std::sort(candidates.begin(), candidates.end(),
//             [](const DetectionPoint &a, const DetectionPoint &b) { 
//                 return a.count > b.count; 
//             });

//         std::vector<DetectionPoint> clustered;
//         for (const auto& cand : candidates) {
//             bool merged = false;
//             for (auto& existing : clustered) {
//                 if (std::hypot(cand.x - existing.x, cand.y - existing.y) < 1.0) {
//                     double w1 = existing.count;
//                     double w2 = cand.count;
//                     existing.x = (existing.x * w1 + cand.x * w2) / (w1 + w2);
//                     existing.y = (existing.y * w1 + cand.y * w2) / (w1 + w2);
//                     existing.count += cand.count;
//                     merged = true;
//                     break;
//                 }
//             }
//             if (!merged) {
//                 clustered.push_back(cand);
//             }
//         }

//         std::sort(clustered.begin(), clustered.end(),
//             [](const DetectionPoint &a, const DetectionPoint &b) { 
//                 return a.count > b.count; 
//             });

//         RCLCPP_INFO(this->get_logger(), "Found %zu potential human locations", clustered.size());

//         if (!h1_present && !clustered.empty()) {
//             RCLCPP_INFO(this->get_logger(), 
//                 "=> Human 1 NEW LOCATION: (%.2f, %.2f) [%d detections]",
//                 clustered[0].x, clustered[0].y, clustered[0].count);
//         }

//         if (!h2_present) {
//             size_t idx = h1_present ? 0 : 1;
//             if (idx < clustered.size()) {
//                 RCLCPP_INFO(this->get_logger(), 
//                     "=> Human 2 NEW LOCATION: (%.2f, %.2f) [%d detections]",
//                     clustered[idx].x, clustered[idx].y, clustered[idx].count);
//             }
//         }
        
//         RCLCPP_INFO(this->get_logger(), "==========================================");
//     }

// private:
//     std::mutex data_mutex_;
//     std::mutex pose_mutex_;

//     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
//     rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

//     nav_msgs::msg::OccupancyGrid map_;
//     std::atomic<bool> have_map_{false};
//     geometry_msgs::msg::Pose current_pose_;

//     tf2_ros::Buffer tf_buffer_;
//     tf2_ros::TransformListener tf_listener_;

//     std::map<std::pair<int,int>, int> all_obstacles_;
//     std::vector<std::pair<double, double>> known_obstacles_;

//     const double human1_x_ = 1.00;
//     const double human1_y_ = -1.00;
//     const double human2_x_ = -12.00;
//     const double human2_y_ = 15.00;
    
//     std::atomic<int> h1_detections_{0};
//     std::atomic<int> h2_detections_{0};
//     std::atomic<int> scans_near_h1_{0};
//     std::atomic<int> scans_near_h2_{0};

//     void initializeKnownObstacles() {
//         known_obstacles_ = {
//             {-8.5, -13.0}, {6.5, -13.0}, {-1.5, -13.0},
//             {13.5, 4.5}, {10.0, 4.5}, {13.5, -21.0}, {13.5, -15.0},
//             {0.4, -2.0}, {3.5, 9.5}, {-1.3, 18.5},
//             {-10.0, 21.5}, {-7.0, 23.6}, {-4.0, 21.5},
//             {-10.4, 14.75}, {-10.4, 10.5}, {-10.4, 6.5}, {-12.85, 4.85},
//             {14.3, -5.5}, {14.3, -4.0},
//             {-11.5, 6.4}, {-14.0, 6.5}, {-12.7, 6.5},
//         };
//     }

//     bool isKnownStaticObstacle(double x, double y) const {
//         for (const auto& [ox, oy] : known_obstacles_) {
//             if (std::hypot(x - ox, y - oy) < 1.5) return true;
//         }
//         return false;
//     }

//     void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
//         std::lock_guard<std::mutex> lock(pose_mutex_);
//         current_pose_ = msg->pose.pose;
//     }

//     void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
//         std::lock_guard<std::mutex> lock(data_mutex_);
//         map_ = *msg;
//         have_map_.store(true);
//         RCLCPP_INFO(this->get_logger(), "Map received: %ux%u @ %.3fm/px", 
//             map_.info.width, map_.info.height, map_.info.resolution);
//     }

//     void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
//         geometry_msgs::msg::TransformStamped tf;
        
//         try {
//             tf = tf_buffer_.lookupTransform("map", scan->header.frame_id, tf2::TimePointZero);
//         } catch (const tf2::TransformException &ex) {
//             return;
//         }

//         if (!have_map_.load()) return;

//         double robot_x = tf.transform.translation.x;
//         double robot_y = tf.transform.translation.y;
        
//         bool near_h1 = std::hypot(robot_x - human1_x_, robot_y - human1_y_) < 2.5;
//         bool near_h2 = std::hypot(robot_x - human2_x_, robot_y - human2_y_) < 2.5;
        
//         if (near_h1) scans_near_h1_++;
//         if (near_h2) scans_near_h2_++;
        
//         bool h1_detected = false;
//         bool h2_detected = false;

//         for (size_t i = 0; i < scan->ranges.size(); ++i) {
//             float r = scan->ranges[i];
//             if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) continue;

//             float angle = scan->angle_min + i * scan->angle_increment;

//             geometry_msgs::msg::PointStamped p_laser, p_map;
//             p_laser.header = scan->header;
//             p_laser.point.x = r * std::cos(angle);
//             p_laser.point.y = r * std::sin(angle);
//             p_laser.point.z = 0.0;

//             tf2::doTransform(p_laser, p_map, tf);
            
//             double px = p_map.point.x;
//             double py = p_map.point.y;

//             if (near_h1 && std::hypot(px - human1_x_, py - human1_y_) < 0.8) {
//                 h1_detected = true;
//             }
//             if (near_h2 && std::hypot(px - human2_x_, py - human2_y_) < 0.8) {
//                 h2_detected = true;
//             }

//             if (r < 3.0) {
//                 std::lock_guard<std::mutex> lock(data_mutex_);
//                 int gx = static_cast<int>(std::round(px * 10.0));
//                 int gy = static_cast<int>(std::round(py * 10.0));
//                 all_obstacles_[{gx, gy}]++;
//             }
//         }

//         if (h1_detected) h1_detections_++;
//         if (h2_detected) h2_detections_++;
//     }
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
    
//     auto detector = std::make_shared<HumanDetector>();
//     Navigator navigator(true);

//     std::thread spin_thread([&]() { rclcpp::spin(detector); });

//     std::cout << "\n[Main] Waiting for map..." << std::endl;
//     while (rclcpp::ok() && !detector->hasMap()) {
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }
//     std::cout << "[Main] Map received!\n" << std::endl;
//     std::this_thread::sleep_for(std::chrono::seconds(2));

//     detector->resetDetection();

//     auto init_pose = std::make_shared<geometry_msgs::msg::Pose>();
//     init_pose->position.x = 2.12;
//     init_pose->position.y = -21.3;
//     init_pose->orientation.z = 0.7071;
//     init_pose->orientation.w = 0.7071;

//     navigator.SetInitialPose(init_pose);
//     navigator.WaitUntilNav2Active();

//     std::vector<geometry_msgs::msg::PoseStamped> waypoints;
//     std::vector<std::string> waypoint_notes;
    
//     auto add_wp = [&](double x, double y, double yaw_deg, const std::string& note = "") {
//         geometry_msgs::msg::PoseStamped p;
//         p.header.frame_id = "map";
//         p.pose.position.x = x;
//         p.pose.position.y = y;
//         double rad = yaw_deg * M_PI / 180.0;
//         p.pose.orientation.z = std::sin(rad / 2.0);
//         p.pose.orientation.w = std::cos(rad / 2.0);
//         waypoints.push_back(p);
//         waypoint_notes.push_back(note);
//     };

//     std::cout << "========================================" << std::endl;
//     std::cout << "   MAP-OPTIMIZED COVERAGE WAYPOINTS" << std::endl;
//     std::cout << "========================================" << std::endl;
//     std::cout << "Map: 1006×1674 @ 0.03m/px" << std::endl;
//     std::cout << "Area: 30m × 50m warehouse" << std::endl;
//     std::cout << "Strategy: Corridor-following coverage" << std::endl;
//     std::cout << "========================================\n" << std::endl;
    
//     // ================================================================
//     // OPTIMAL WAYPOINTS - DESIGNED FROM ACTUAL MAP
//     // Based on navigable corridors visible in my_warehouse_map.jpg
//     // ================================================================
    
//     std::cout << "Generating optimal waypoints:\n" << std::endl;
    
//     // SOUTH SECTION - Between 3 vertical shelves
//     add_wp(7.0, -17.0, 90.0, "South-East corridor");
//     add_wp(3.0, -15.0, 90.0, "East-Center shelf corridor");
//     add_wp(-2.0, -15.0, 90.0, "Center-West shelf corridor");
//     add_wp(-8.0, -17.0, 90.0, "South-West corridor");
    
//     // CENTER AREA - H1 vicinity and open space
//     add_wp(-5.0, -5.0, 90.0, "West-Center area");
//     add_wp(1.0, -1.0, 90.0, "★ H1 ORIGINAL (1, -1)");
//     add_wp(5.0, -5.0, 90.0, "East-Center area");
//     add_wp(0.0, 5.0, 90.0, "Center open area");
    
//     // NORTH SECTION - Above horizontal shelves
//     add_wp(8.0, 12.0, 90.0, "North-East area");
//     add_wp(-10.0, 15.0, 90.0, "West corridor (H2 vicinity)");
//     add_wp(-12.0, 18.0, 90.0, "★ H2 ORIGINAL (-12, 15)");
//     add_wp(0.0, 18.0, 90.0, "North-Center");
    
//     // EDGE CHECKS - For humans moved to corners
//     add_wp(10.0, 5.0, 90.0, "East edge mid");
//     add_wp(10.0, -10.0, 90.0, "East edge south");
    
//     std::cout << "[Main] Generated " << waypoints.size() << " waypoints\n" << std::endl;
    
//     // Print waypoint plan
//     for (size_t i = 0; i < waypoints.size(); ++i) {
//         std::cout << "  WP" << (i+1) << ": (" 
//                   << waypoints[i].pose.position.x << ", " 
//                   << waypoints[i].pose.position.y << ") - "
//                   << waypoint_notes[i] << std::endl;
//     }
    
//     // Identify spin locations (H1 and H2)
//     std::vector<int> spin_at;
//     for (size_t i = 0; i < waypoints.size(); ++i) {
//         double x = waypoints[i].pose.position.x;
//         double y = waypoints[i].pose.position.y;
//         if ((std::hypot(x - 1.0, y + 1.0) < 1.0) ||  // H1
//             (std::hypot(x + 12.0, y - 15.0) < 4.0)) { // H2
//             spin_at.push_back(i);
//         }
//     }
    
//     std::cout << "\n[Main] Will perform 360° spin at " << spin_at.size() 
//               << " strategic locations (H1 & H2 originals)\n" << std::endl;
//     std::cout << "Estimated completion time: 10-15 minutes\n" << std::endl;
//     std::cout << "========================================\n" << std::endl;

//     // Execute navigation
//     int success = 0, failed = 0;
//     auto start = std::chrono::steady_clock::now();
    
//     for (size_t i = 0; i < waypoints.size(); ++i) {
//         auto goal = std::make_shared<geometry_msgs::msg::Pose>(waypoints[i].pose);
        
//         auto elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(
//             std::chrono::steady_clock::now() - start).count();
//         int mins = elapsed_sec / 60;
//         int secs = elapsed_sec % 60;
        
//         std::cout << "[WP " << (i+1) << "/" << waypoints.size() << "] ";
//         std::cout << waypoint_notes[i];
//         std::cout << " | " << mins << ":" << (secs < 10 ? "0" : "") << secs << " elapsed" << std::endl;
        
//         navigator.GoToPose(goal);
        
//         while (rclcpp::ok() && !navigator.IsTaskComplete()) {
//             std::this_thread::sleep_for(std::chrono::milliseconds(100));
//         }

//         if (navigator.GetResult() == rclcpp_action::ResultCode::SUCCEEDED) {
//             std::cout << "  ✓ Reached successfully" << std::endl;
//             success++;
            
//             // 360° spin at H1/H2 locations
//             if (std::find(spin_at.begin(), spin_at.end(), i) != spin_at.end()) {
//                 std::cout << "  → Performing 360° scan..." << std::endl;
//                 navigator.Spin();
//                 while (rclcpp::ok() && !navigator.IsTaskComplete()) {
//                     std::this_thread::sleep_for(std::chrono::milliseconds(100));
//                 }
//                 std::this_thread::sleep_for(std::chrono::milliseconds(200));
//             }
//         } else {
//             std::cout << "  ✗ Navigation failed" << std::endl;
//             failed++;
//             if (failed > 3) {
//                 std::cerr << "\n[ERROR] Too many failures (" << failed << "), aborting mission." << std::endl;
//                 break;
//             }
//         }
//     }

//     auto runtime = std::chrono::duration_cast<std::chrono::seconds>(
//         std::chrono::steady_clock::now() - start).count();
//     int runtime_min = runtime / 60;
//     int runtime_sec = runtime % 60;

//     std::cout << "\n========================================" << std::endl;
//     std::cout << "     WAREHOUSE COVERAGE COMPLETE" << std::endl;
//     std::cout << "========================================" << std::endl;
//     std::cout << "Waypoints completed: " << success << "/" << waypoints.size() 
//               << " (" << (waypoints.size() > 0 ? (100*success/waypoints.size()) : 0) << "%)" << std::endl;
//     std::cout << "Failed attempts: " << failed << std::endl;
//     std::cout << "Total runtime: " << runtime_min << "m " << runtime_sec << "s" << std::endl;
//     std::cout << "========================================\n" << std::endl;
    
//     detector->reportHumans();
    
//     rclcpp::shutdown();
//     if (spin_thread.joinable()) spin_thread.join();
    
//     return 0;
// }



// #include <iostream>
// #include <thread>
// #include <chrono>
// #include <vector>
// #include <map>
// #include <cmath>
// #include <algorithm>
// #include <mutex>
// #include <atomic>

// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"
// #include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
// #include "geometry_msgs/msg/point_stamped.hpp"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include "navigation/navigation.hpp"

// using namespace std::chrono_literals;

// // Structure for detected clusters
// struct DetectedCluster {
//     double x, y;
//     int detection_count;
//     double confidence;
// };

// class HumanDetector : public rclcpp::Node {
// public:
//     HumanDetector() : Node("human_detector_node"), 
//                       tf_buffer_(this->get_clock()), 
//                       tf_listener_(tf_buffer_) {
        
//         if (!this->has_parameter("use_sim_time")) {
//             this->declare_parameter("use_sim_time", true);
//         }
        
//         initKnownObstacles();
        
//         // Subscribers
//         map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//             "/map", rclcpp::QoS(1).transient_local().reliable(),
//             std::bind(&HumanDetector::mapCallback, this, std::placeholders::_1));
            
//         scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//             "/scan", rclcpp::SensorDataQoS(),
//             std::bind(&HumanDetector::scanCallback, this, std::placeholders::_1));
            
//         amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
//             "/amcl_pose", 10,
//             std::bind(&HumanDetector::amclCallback, this, std::placeholders::_1));
        
//         RCLCPP_INFO(this->get_logger(), "Human Detector Initialized.");
//     }
    
//     bool isLocalized() const { return is_localized_; }
//     bool hasMap() const { return has_map_; }
    
//     // Check if human is at original position
//     bool isHumanAtPosition(int human_id, double x, double y) {
//         std::lock_guard<std::mutex> lock(detection_mutex_);
        
//         if (human_id == 1) {
//             return h1_present_count_ >= 8 && h1_absent_count_ <= 3;
//         } else {
//             return h2_present_count_ >= 8 && h2_absent_count_ <= 3;
//         }
//     }
    
//     // Get new location if human moved
//     std::pair<double, double> getNewLocation(int human_id) {
//         std::lock_guard<std::mutex> lock(detection_mutex_);
        
//         // Find best candidate cluster far from original positions
//         std::vector<DetectedCluster> candidates;
        
//         for (const auto& [key, count] : obstacle_detections_) {
//             if (count < 10) continue;
            
//             double px = key.first * 0.1;
//             double py = key.second * 0.1;
            
//             // Skip if near original positions
//             double d1 = std::hypot(px - H1_ORIG_X, py - H1_ORIG_Y);
//             double d2 = std::hypot(px - H2_ORIG_X, py - H2_ORIG_Y);
//             if (d1 < 1.5 || d2 < 1.5) continue;
            
//             // Skip if known static obstacle
//             if (isNearStaticObstacle(px, py)) continue;
            
//             // Skip if out of bounds
//             if (px < -13 || px > 13 || py < -23 || py > 23) continue;
            
//             candidates.push_back({px, py, count, (double)count});
//         }
        
//         // Cluster nearby detections
//         std::vector<DetectedCluster> clustered;
//         for (const auto& cand : candidates) {
//             bool merged = false;
//             for (auto& existing : clustered) {
//                 if (std::hypot(cand.x - existing.x, cand.y - existing.y) < 0.8) {
//                     double total = existing.detection_count + cand.detection_count;
//                     existing.x = (existing.x * existing.detection_count + cand.x * cand.detection_count) / total;
//                     existing.y = (existing.y * existing.detection_count + cand.y * cand.detection_count) / total;
//                     existing.detection_count += cand.detection_count;
//                     existing.confidence = total;
//                     merged = true;
//                     break;
//                 }
//             }
//             if (!merged) {
//                 clustered.push_back(cand);
//             }
//         }
        
//         // Sort by confidence
//         std::sort(clustered.begin(), clustered.end(), 
//                  [](const DetectedCluster& a, const DetectedCluster& b) {
//                      return a.confidence > b.confidence;
//                  });
        
//         // Return best candidate
//         if (human_id == 1 && !clustered.empty()) {
//             return {clustered[0].x, clustered[0].y};
//         } else if (human_id == 2 && clustered.size() > 1) {
//             return {clustered[1].x, clustered[1].y};
//         } else if (human_id == 2 && clustered.size() == 1) {
//             return {clustered[0].x, clustered[0].y};
//         }
        
//         return {0, 0};
//     }
    
//     void resetDetections() {
//         std::lock_guard<std::mutex> lock(detection_mutex_);
//         h1_present_count_ = 0;
//         h1_absent_count_ = 0;
//         h2_present_count_ = 0;
//         h2_absent_count_ = 0;
//         h1_scans_near_ = 0;
//         h2_scans_near_ = 0;
//         obstacle_detections_.clear();
//     }

// private:
//     // ROS subscribers
//     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
//     rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    
//     // TF
//     tf2_ros::Buffer tf_buffer_;
//     tf2_ros::TransformListener tf_listener_;
    
//     // Map
//     nav_msgs::msg::OccupancyGrid current_map_;
//     bool has_map_ = false;
//     bool is_localized_ = false;
//     std::mutex map_mutex_;
//     std::mutex detection_mutex_;
    
//     // Known obstacles from environment
//     std::vector<std::pair<double, double>> static_obstacles_;
    
//     // Original human locations
//     const double H1_ORIG_X = 1.0;
//     const double H1_ORIG_Y = -1.0;
//     const double H2_ORIG_X = -12.0;
//     const double H2_ORIG_Y = 15.0;
    
//     // Detection counters
//     int h1_present_count_ = 0;
//     int h1_absent_count_ = 0;
//     int h2_present_count_ = 0;
//     int h2_absent_count_ = 0;
//     int h1_scans_near_ = 0;
//     int h2_scans_near_ = 0;
    
//     // Obstacle detection map (for finding new positions)
//     std::map<std::pair<int, int>, int> obstacle_detections_;
    
//     void initKnownObstacles() {
//         static_obstacles_ = {
//             {-8.5, -13}, {6.5, -13}, {-1.5, -13},
//             {13.5, 4.5}, {10, 4.5}, {13.5, -21}, {13.5, -15},
//             {0.4, -2}, {3.5, 9.5}, {-1.3, 18.5},
//             {-10, 21.5}, {-7, 23.6}, {-4, 21.5},
//             {-10.4, 14.75}, {-10.4, 10.5}, {-10.4, 6.5}, {-12.85, 4.85},
//             {14.3, -5.5}, {14.3, -4},
//             {-11.5, 6.4}, {-14, 6.5}, {-12.7, 6.5}
//         };
//     }
    
//     bool isNearStaticObstacle(double x, double y) {
//         for (const auto& obs : static_obstacles_) {
//             if (std::hypot(x - obs.first, y - obs.second) < 1.2) return true;
//         }
//         return false;
//     }
    
//     void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
//         std::lock_guard<std::mutex> lock(map_mutex_);
//         current_map_ = *msg;
//         has_map_ = true;
//         RCLCPP_INFO_ONCE(this->get_logger(), "Map received! Grid resolution: %.3f", 
//                         current_map_.info.resolution);
//     }
    
//     void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
//         if (!is_localized_) {
//             is_localized_ = true;
//             RCLCPP_INFO(this->get_logger(), "AMCL Pose received. Localization active.");
//         }
//     }
    
//     void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
//         if (!has_map_ || !is_localized_) return;
        
//         // Get transform
//         geometry_msgs::msg::TransformStamped tf_stamped;
//         try {
//             tf_stamped = tf_buffer_.lookupTransform("map", msg->header.frame_id, tf2::TimePointZero);
//         } catch (tf2::TransformException &ex) {
//             return;
//         }
        
//         double robot_x = tf_stamped.transform.translation.x;
//         double robot_y = tf_stamped.transform.translation.y;
        
//         // Check if near human positions (for targeted checking)
//         bool near_h1 = std::hypot(robot_x - H1_ORIG_X, robot_y - H1_ORIG_Y) < 3.5;
//         bool near_h2 = std::hypot(robot_x - H2_ORIG_X, robot_y - H2_ORIG_Y) < 3.5;
        
//         std::lock_guard<std::mutex> lock(detection_mutex_);
        
//         if (near_h1) h1_scans_near_++;
//         if (near_h2) h2_scans_near_++;
        
//         bool h1_detected_this_scan = false;
//         bool h2_detected_this_scan = false;
        
//         double angle = msg->angle_min;
//         for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment) {
//             float r = msg->ranges[i];
            
//             if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max || r > 8.0) continue;
            
//             // Transform point to map frame
//             geometry_msgs::msg::PointStamped p_laser, p_map;
//             p_laser.header = msg->header;
//             p_laser.point.x = r * std::cos(angle);
//             p_laser.point.y = r * std::sin(angle);
//             p_laser.point.z = 0.0;
            
//             tf2::doTransform(p_laser, p_map, tf_stamped);
            
//             double px = p_map.point.x;
//             double py = p_map.point.y;
            
//             // Check if detection is at original human positions
//             if (near_h1) {
//                 double dist_h1 = std::hypot(px - H1_ORIG_X, py - H1_ORIG_Y);
//                 if (dist_h1 < 0.6) {
//                     h1_detected_this_scan = true;
//                 }
//             }
            
//             if (near_h2) {
//                 double dist_h2 = std::hypot(px - H2_ORIG_X, py - H2_ORIG_Y);
//                 if (dist_h2 < 0.6) {
//                     h2_detected_this_scan = true;
//                 }
//             }
            
//             // Record all obstacles for later analysis
//             if (r < 5.0) {
//                 int grid_x = static_cast<int>(std::round(px * 10.0));
//                 int grid_y = static_cast<int>(std::round(py * 10.0));
//                 obstacle_detections_[{grid_x, grid_y}]++;
//             }
//         }
        
//         // Update presence counters
//         if (near_h1) {
//             if (h1_detected_this_scan) {
//                 h1_present_count_++;
//             } else {
//                 h1_absent_count_++;
//             }
//         }
        
//         if (near_h2) {
//             if (h2_detected_this_scan) {
//                 h2_present_count_++;
//             } else {
//                 h2_absent_count_++;
//             }
//         }
//     }
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
    
//     auto detector = std::make_shared<HumanDetector>();
//     Navigator navigator(true);
    
//     std::thread detector_thread([&]() { rclcpp::spin(detector); });
    
//     std::cout << "\n[Main] Setting initial pose..." << std::endl;
//     auto init_pose = std::make_shared<geometry_msgs::msg::Pose>();
//     init_pose->position.x = 2.12;
//     init_pose->position.y = -21.3;
//     init_pose->orientation.z = 0.7071;
//     init_pose->orientation.w = 0.7071;
    
//     std::cout << "[Main] Waiting for Nav2..." << std::endl;
//     navigator.SetInitialPose(init_pose);
//     navigator.WaitUntilNav2Active();
    
//     std::cout << "[Main] Waiting for map..." << std::endl;
//     while (rclcpp::ok() && !detector->hasMap()) {
//         std::this_thread::sleep_for(100ms);
//     }
    
//     std::cout << "[Main] Map received!\n" << std::endl;
//     std::this_thread::sleep_for(1s);
    
//     // Define systematic waypoints for coverage (MRTP-style grid coverage)
//     struct Waypoint { double x, y, yaw; std::string note; };
//     std::vector<Waypoint> waypoints = {
//         // Visit H1 original location
//         {1.0, -1.0, 90.0, "★ H1 ORIGINAL LOCATION"},
//         {1.5, -2.0, 90.0, "H1 verification"},
        
//         // Visit H2 original location
//         {-12.0, 15.0, 90.0, "★ H2 ORIGINAL LOCATION"},
//         {-11.0, 16.0, 90.0, "H2 verification"},
        
//         // Systematic coverage
//         {7.0, -17.0, 90.0, "Southeast corridor"},
//         {3.0, -15.0, 90.0, "Center-south"},
//         {-2.0, -15.0, 90.0, "Southwest corridor"},
//         {-5.0, -5.0, 90.0, "West-center"},
//         {5.0, -5.0, 90.0, "East-center"},
//         {0.0, 5.0, 90.0, "Central area"},
//         {8.0, 12.0, 90.0, "Northeast"},
//         {-10.0, 15.0, 90.0, "Northwest"},
//         {0.0, 18.0, 90.0, "North-center"},
//         {10.0, 5.0, 90.0, "East edge"},
//         {10.0, -10.0, 90.0, "Southeast edge"}
//     };
    
//     // Execute waypoint navigation
//     for (size_t i = 0; i < waypoints.size(); ++i) {
//         if (!rclcpp::ok()) break;
        
//         auto& wp = waypoints[i];
//         std::cout << "[Main] Going to WP" << (i+1) << ": " << wp.note << std::endl;
        
//         auto goal = std::make_shared<geometry_msgs::msg::Pose>();
//         goal->position.x = wp.x;
//         goal->position.y = wp.y;
//         double rad = wp.yaw * M_PI / 180.0;
//         goal->orientation.z = std::sin(rad / 2.0);
//         goal->orientation.w = std::cos(rad / 2.0);
        
//         navigator.GoToPose(goal);
        
//         while (rclcpp::ok() && !navigator.IsTaskComplete()) {
//             std::this_thread::sleep_for(100ms);
//         }
        
//         if (navigator.GetResult() == rclcpp_action::ResultCode::SUCCEEDED) {
//             std::cout << "   reached." << std::endl;
            
//             // Spin at human original locations
//             if (wp.note.find("★") != std::string::npos) {
//                 std::cout << "   Performing 360° scan..." << std::endl;
//                 detector->resetDetections();
//                 navigator.Spin();
//                 while (rclcpp::ok() && !navigator.IsTaskComplete()) {
//                     std::this_thread::sleep_for(100ms);
//                 }
//                 std::this_thread::sleep_for(500ms);
//             }
//         } else {
//             std::cout << "   failed." << std::endl;
//         }
//     }
    
//     // Final report
//     std::cout << "\n========================================" << std::endl;
//     std::cout << "       HUMAN DETECTION REPORT" << std::endl;
//     std::cout << "========================================\n" << std::endl;
    
//     bool h1_present = detector->isHumanAtPosition(1, 1.0, -1.0);
//     bool h2_present = detector->isHumanAtPosition(2, -12.0, 15.0);
    
//     if (h1_present) {
//         std::cout << "[Human 1] STILL AT ORIGINAL (1.0, -1.0) ✓" << std::endl;
//     } else {
//         std::cout << "[Human 1] MOVED from (1.0, -1.0)" << std::endl;
//         auto new_loc = detector->getNewLocation(1);
//         std::cout << "          NEW LOCATION: (" << new_loc.first << ", " << new_loc.second << ")" << std::endl;
//     }
    
//     if (h2_present) {
//         std::cout << "[Human 2] STILL AT ORIGINAL (-12.0, 15.0) ✓" << std::endl;
//     } else {
//         std::cout << "[Human 2] MOVED from (-12.0, 15.0)" << std::endl;
//         auto new_loc = detector->getNewLocation(2);
//         std::cout << "          NEW LOCATION: (" << new_loc.first << ", " << new_loc.second << ")" << std::endl;
//     }
    
//     std::cout << "\n========================================\n" << std::endl;
    
//     rclcpp::shutdown();
//     if (detector_thread.joinable()) detector_thread.join();
    
//     return 0;
// }



// =============================================================================================
// MRTP FINAL PROJECT: AUTONOMOUS HUMAN DETECTOR
// Features: Map-based coverage, Nearest Neighbor path planning, Dynamic Obstacle Verification
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

// Structure to cluster laser hits for final reporting
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
        // Use simulation time for correct TF lookups in Gazebo
        if (!this->has_parameter("use_sim_time")) {
            this->declare_parameter("use_sim_time", true);
        }
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        // Subscribe to map
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local().reliable(),
            std::bind(&HumanDetector::mapCallback, this, std::placeholders::_1));

        // Subscribe to laser scan
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&HumanDetector::scanCallback, this, std::placeholders::_1));

        // Subscribe to AMCL pose (Localization)
        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&HumanDetector::amclCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Autonomous Human Detector Initialized.");
    }

    bool hasMap() const { return have_map_.load(); }
    bool isLocalized() const { return is_localized_.load(); }

    // Helper to check map value at world coordinates (Thread-Safe)
    // Returns: -1 (Unknown/Error), 0 (Free), 100 (Occupied)
    int8_t getMapValue(double x, double y) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!have_map_) return -1;

        // Convert Metric -> Grid
        int grid_x = static_cast<int>((x - map_.info.origin.position.x) / map_.info.resolution);
        int grid_y = static_cast<int>((y - map_.info.origin.position.y) / map_.info.resolution);

        if (grid_x < 0 || grid_x >= (int)map_.info.width || 
            grid_y < 0 || grid_y >= (int)map_.info.height) {
            return -1; 
        }

        return map_.data[grid_y * map_.info.width + grid_x];
    }

    // ---------------------------------------------------------
    // AUTONOMOUS PATH GENERATION
    // Scans the map metadata and generates goals in free space.
    // ---------------------------------------------------------
    std::vector<geometry_msgs::msg::PoseStamped> generateCoveragePath() {
        std::vector<geometry_msgs::msg::PoseStamped> goals;
        
        // LOCK MUTEX ONCE HERE
        std::lock_guard<std::mutex> lock(map_mutex_); 
        
        if (!have_map_) return goals;

        // How far apart should waypoints be? 4.5m is good for laser range (~8m)
        double step_size = 4.5; 
        
        // Define a lambda helper to access map data directly WITHOUT locking again
        // (This prevents the deadlock you experienced)
        auto is_free_unsafe = [&](double wx, double wy) {
            int grid_x = static_cast<int>((wx - map_.info.origin.position.x) / map_.info.resolution);
            int grid_y = static_cast<int>((wy - map_.info.origin.position.y) / map_.info.resolution);

            if (grid_x < 0 || grid_x >= (int)map_.info.width || 
                grid_y < 0 || grid_y >= (int)map_.info.height) {
                return false; 
            }
            // 0 means Free Space
            return map_.data[grid_y * map_.info.width + grid_x] == 0;
        };

        // Iterate through the map bounds
        for (double y = map_.info.origin.position.y + 2.0; 
             y < (map_.info.origin.position.y + (map_.info.height * map_.info.resolution)); 
             y += step_size) {
            
            for (double x = map_.info.origin.position.x + 2.0; 
                 x < (map_.info.origin.position.x + (map_.info.width * map_.info.resolution)); 
                 x += step_size) {

                // Check point and slight surroundings to ensure safety
                if (is_free_unsafe(x, y) && 
                    is_free_unsafe(x + 0.5, y) && 
                    is_free_unsafe(x - 0.5, y)) {
                    
                    geometry_msgs::msg::PoseStamped p;
                    p.header.frame_id = "map";
                    p.pose.position.x = x;
                    p.pose.position.y = y;
                    p.pose.orientation.w = 1.0;
                    goals.push_back(p);
                }
            }
        }

        // OPTIMIZATION: Sort points using Nearest Neighbor (Greedy)
        // This ensures the robot moves to the closest point next, rather than zigzagging wildly.
        if (goals.empty()) return goals;

        double current_x = current_pose_.position.x;
        double current_y = current_pose_.position.y;
        
        std::vector<geometry_msgs::msg::PoseStamped> sorted_goals;
        std::vector<geometry_msgs::msg::PoseStamped> remaining = goals;

        while (!remaining.empty()) {
            auto nearest_it = remaining.begin();
            double min_dist = std::numeric_limits<double>::max();

            for (auto it = remaining.begin(); it != remaining.end(); ++it) {
                double d = std::hypot(it->pose.position.x - current_x, 
                                      it->pose.position.y - current_y);
                if (d < min_dist) {
                    min_dist = d;
                    nearest_it = it;
                }
            }

            sorted_goals.push_back(*nearest_it);
            current_x = nearest_it->pose.position.x;
            current_y = nearest_it->pose.position.y;
            remaining.erase(nearest_it);
        }

        return sorted_goals;
    }

    // ---------------------------------------------------------
    // REPORT GENERATION
    // Analyzes accumulated sensor data to determine human positions
    // ---------------------------------------------------------
    void reportFindings() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // Define Original Locations (From project description/map)
        const double H1_X = 1.0, H1_Y = -1.0;
        const double H2_X = -12.0, H2_Y = 15.0;

        // 1. Determine Presence (Did they move?)
        // Heuristic: If we scanned the area enough times, did we get enough hits?
        bool h1_present = (scans_near_h1_ > 20 && ((double)h1_hits_ / scans_near_h1_) > 0.1);
        bool h2_present = (scans_near_h2_ > 20 && ((double)h2_hits_ / scans_near_h2_) > 0.1);

        // 2. Find New Locations (Dynamic Obstacles)
        // We look for dynamic obstacles: Laser Hit exists, but Map says FREE
        std::vector<DetectionCluster> candidates;
        
        for (const auto& [coord, count] : dynamic_obstacles_) {
            if (count < 15) continue; // Filter noise

            double wx = coord.first / 10.0; // Decode 10cm grid key
            double wy = coord.second / 10.0;

            // Ignore if near original spots (already handled by presence check)
            if (std::hypot(wx - H1_X, wy - H1_Y) < 1.5) continue;
            if (std::hypot(wx - H2_X, wy - H2_Y) < 1.5) continue;

            // Ignore boundaries/walls 
            // (The dynamic check in scanCallback filters map walls, but this is a sanity check)
            candidates.push_back({wx, wy, count});
        }

        // 3. Cluster nearby points
        std::vector<DetectionCluster> final_humans;
        for (const auto& cand : candidates) {
            bool merged = false;
            for (auto& f : final_humans) {
                if (std::hypot(f.x - cand.x, f.y - cand.y) < 1.0) {
                    // Average the coordinates
                    f.x = (f.x + cand.x) / 2.0;
                    f.y = (f.y + cand.y) / 2.0;
                    f.count += cand.count;
                    merged = true;
                    break;
                }
            }
            if (!merged) final_humans.push_back(cand);
        }

        // Sort by detection confidence
        std::sort(final_humans.begin(), final_humans.end(), 
            [](const DetectionCluster &a, const DetectionCluster &b) { return a.count > b.count; });

        std::cout << "\n========================================" << std::endl;
        std::cout << "       HUMAN DETECTION REPORT" << std::endl;
        std::cout << "========================================" << std::endl;

        if (h1_present) {
            std::cout << "[Human 1] STILL AT ORIGINAL (" << H1_X << ", " << H1_Y << ") [DETECTED]" << std::endl;
        } else {
            std::cout << "[Human 1] MOVED from (" << H1_X << ", " << H1_Y << ")" << std::endl;
            if (!final_humans.empty()) {
                std::cout << "          NEW LOCATION: (" << final_humans[0].x << ", " << final_humans[0].y << ")" << std::endl;
            } else {
                std::cout << "          NEW LOCATION: Not found (Check scan coverage)" << std::endl;
            }
        }

        if (h2_present) {
            std::cout << "[Human 2] STILL AT ORIGINAL (" << H2_X << ", " << H2_Y << ") [DETECTED]" << std::endl;
        } else {
            std::cout << "[Human 2] MOVED from (" << H2_X << ", " << H2_Y << ")" << std::endl;
            // If H1 moved, H2 is likely the second cluster found, otherwise the first
            size_t idx = (!h1_present && final_humans.size() > 1) ? 1 : 0;
            if (final_humans.size() > idx) {
                std::cout << "          NEW LOCATION: (" << final_humans[idx].x << ", " << final_humans[idx].y << ")" << std::endl;
            } else {
                std::cout << "          NEW LOCATION: Not found (Check scan coverage)" << std::endl;
            }
        }
        std::cout << "========================================\n" << std::endl;
    }

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

    // Original Human Locations (Known from problem statement)
    const double H1_X = 1.0, H1_Y = -1.0;
    const double H2_X = -12.0, H2_Y = 15.0;

    int scans_near_h1_{0}, h1_hits_{0};
    int scans_near_h2_{0}, h2_hits_{0};

    // Store potential new locations: Key=(GridX, GridY), Value=HitCount
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
        // We need Map and Location before we can process scans meaningfully
        if (!have_map_.load() || !is_localized_.load()) return;

        // [MRTP Ch 4] Get Transform from Laser Frame to Map Frame
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_.lookupTransform("map", scan->header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            return;
        }

        // Current Robot Position from Transform
        double rx = tf.transform.translation.x;
        double ry = tf.transform.translation.y;

        // Check proximity to Original Human locations
        // This helps us decide if we SHOULD have seen them
        bool near_h1 = std::hypot(rx - H1_X, ry - H1_Y) < 4.0;
        bool near_h2 = std::hypot(rx - H2_X, ry - H2_Y) < 4.0;

        if (near_h1) scans_near_h1_++;
        if (near_h2) scans_near_h2_++;

        bool h1_hit_this_scan = false;
        bool h2_hit_this_scan = false;

        // Iterate through laser rays
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float r = scan->ranges[i];
            
            // Filter invalid ranges or extremely far readings
            if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) continue;

            // [MRTP Ch 4] Polar -> Cartesian -> Map Frame Transform
            float angle = scan->angle_min + i * scan->angle_increment;
            geometry_msgs::msg::PointStamped p_laser, p_map;
            p_laser.header = scan->header;
            p_laser.point.x = r * std::cos(angle);
            p_laser.point.y = r * std::sin(angle);
            p_laser.point.z = 0.0;

            tf2::doTransform(p_laser, p_map, tf);
            
            double wx = p_map.point.x;
            double wy = p_map.point.y;

            // Check if this laser hit is ON an original human spot
            if (near_h1 && std::hypot(wx - H1_X, wy - H1_Y) < 0.5) h1_hit_this_scan = true;
            if (near_h2 && std::hypot(wx - H2_X, wy - H2_Y) < 0.5) h2_hit_this_scan = true;

            // [MRTP Ch 9] Dynamic Obstacle Check
            // Logic: If map says FREE (0) but laser says HIT -> It's a moved human
            int map_val = getMapValue(wx, wy);
            
            // We verify map_val is 0 (Free) and range is reasonable (<5m for accuracy)
            if (map_val == 0 && r < 5.0) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                // Bucket coordinates into 10cm cells to accumulate hits
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
    
    // Create Node and Navigation Interface
    auto detector = std::make_shared<HumanDetector>();
    Navigator navigator(true); // 'true' enables verbose logging

    // Spin detector in a background thread so callbacks (Scan, TF, Map) run constantly
    std::thread spin_thread([&]() { rclcpp::spin(detector); });

    // 1. Wait for Map
    std::cout << "\n[Main] Waiting for Map..." << std::endl;
    while (rclcpp::ok() && !detector->hasMap()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "[Main] Map Received." << std::endl;

    // 2. Initialize Pose (Required for Nav2 to start)
    auto init_pose = std::make_shared<geometry_msgs::msg::Pose>();
    init_pose->position.x = 2.12;
    init_pose->position.y = -21.3;
    init_pose->orientation.z = 0.7071;
    init_pose->orientation.w = 0.7071;
    
    std::cout << "[Main] Initializing Pose..." << std::endl;
    navigator.SetInitialPose(init_pose);
    navigator.WaitUntilNav2Active();

    // 3. Generate Autonomous Coverage Path
    std::cout << "[Main] Generating Autonomous Coverage Path based on Map Free Space..." << std::endl;
    auto waypoints = detector->generateCoveragePath();
    std::cout << "[Main] Generated " << waypoints.size() << " optimized waypoints." << std::endl;

    // 4. Execute Path
    int wp_count = 0;
    for (const auto& wp : waypoints) {
        wp_count++;
        // Skip waypoints extremely close to start to save time
        if (std::hypot(wp.pose.position.x - 2.12, wp.pose.position.y - (-21.3)) < 2.0) continue;

        std::cout << "[Main] Navigating to WP " << wp_count << "/" << waypoints.size() 
                  << " (" << wp.pose.position.x << ", " << wp.pose.position.y << ")..." << std::flush;
        
        auto goal = std::make_shared<geometry_msgs::msg::Pose>(wp.pose);
        navigator.GoToPose(goal);

        while (rclcpp::ok() && !navigator.IsTaskComplete()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (navigator.GetResult() == rclcpp_action::ResultCode::SUCCEEDED) {
            std::cout << " DONE" << std::endl;
        } else {
            std::cout << " SKIPPED (Unreachable)" << std::endl;
        }
    }

    // 5. Final Report
    detector->reportFindings();

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}