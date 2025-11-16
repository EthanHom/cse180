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
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// MRTP navigation library (Navigator wrapper around Nav2)
#include "navigation/navigation.hpp"   // MRTP navigation package

struct Cluster
{
  double sum_x{0.0};
  double sum_y{0.0};
  int count{0};

  double cx() const { return (count > 0) ? sum_x / count : 0.0; }
  double cy() const { return (count > 0) ? sum_y / count : 0.0; }
};

struct HumanFootprint
{
  double x;
  double y;
  double radius;   // radius in meters within which we count hits
  int hits{0};
};

class HumanDetector
{
public:
  HumanDetector()
  : node_(rclcpp::Node::make_shared("tb4_human_detector")),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_, node_, false),
    navigator_(true)   // debug flag as in MRTP navigation example
  {
    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10,
      std::bind(&HumanDetector::mapCallback, this, std::placeholders::_1));

    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&HumanDetector::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "HumanDetector node started.");

    // Start navigation after construction
    setupAndNavigate();
  }

  rclcpp::Node::SharedPtr getNode() { return node_; }

private:
  rclcpp::Node::SharedPtr node_;

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  // Map cache
  nav_msgs::msg::OccupancyGrid map_;
  bool have_map_{false};

  // tf2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // MRTP Navigator wrapper around Nav2
  Navigator navigator_;

  // Clustering data
  std::vector<Cluster> clusters_;
  const double cluster_dist_thresh_ = 0.7;   // meters, cluster radius
  const int    min_points_per_cluster_ = 150; // tune experimentally

  // Human footprints in the original map (approximate coordinates)
  HumanFootprint human1_{1.04,  -0.974, 0.8, 0};
  HumanFootprint human2_{-12.0, 15.20, 0.8, 0};
  // other approximate coordinates:
  // human 1: 1.06, -1
  // human 2: -12, 15.2
  const int human_seen_threshold_ = 40;  // number of laser points to consider "seen"

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = *msg;
    have_map_ = true;
    RCLCPP_INFO(node_->get_logger(), "Map received: %u x %u",
                map_.info.width, map_.info.height);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    if (!have_map_) {
      return;
    }

    // Make sure the transform we need exists, using the latest available time (0)
    // and no timeout, to avoid tf2's dedicated-thread warnings.
    if (!tf_buffer_.canTransform(
          "map", scan->header.frame_id, rclcpp::Time(0))) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "No transform yet between %s and map",
        scan->header.frame_id.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      // Use latest available transform instead of scan->header.stamp
      tf = tf_buffer_.lookupTransform(
        "map", scan->header.frame_id, rclcpp::Time(0));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", ex.what());
      return;
    }

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      float r = scan->ranges[i];
      if (!std::isfinite(r) ||
          r < scan->range_min ||
          r > scan->range_max) {
        continue;
      }

      float angle = scan->angle_min + i * scan->angle_increment;
      geometry_msgs::msg::PointStamped p_laser;
      p_laser.header = scan->header;
      p_laser.point.x = r * std::cos(angle);
      p_laser.point.y = r * std::sin(angle);
      p_laser.point.z = 0.0;

      geometry_msgs::msg::PointStamped p_map;
      tf2::doTransform(p_laser, p_map, tf);

      // Check within map bounds
      int mx, my;
      if (!worldToMap(p_map.point.x, p_map.point.y, mx, my)) {
        continue;
      }

      int idx = my * static_cast<int>(map_.info.width) + mx;
      if (idx < 0 || idx >= static_cast<int>(map_.data.size())) {
        continue;
      }

      int8_t occ = map_.data[idx];  // -1 unknown, 0 free, 100 occupied
      bool map_free = (occ >= 0 && occ < 30);   // stricter free space definition

      // 1) Count hits near original human footprints using *scan* only
      updateHumanHits(p_map.point.x, p_map.point.y);

      // 2) New obstacle in map-free space → candidate dynamic obstacle
      if (map_free) {
        addPointToClusters(p_map.point.x, p_map.point.y);
      }
    }
  }

  bool worldToMap(double wx, double wy, int &mx, int &my) const
  {
    double origin_x = map_.info.origin.position.x;
    double origin_y = map_.info.origin.position.y;
    double res = map_.info.resolution;

    mx = static_cast<int>((wx - origin_x) / res);
    my = static_cast<int>((wy - origin_y) / res);

    if (mx < 0 || my < 0 ||
        mx >= static_cast<int>(map_.info.width) ||
        my >= static_cast<int>(map_.info.height)) {
      return false;
    }
    return true;
  }

  void updateHumanHits(double x, double y)
  {
    // Human 1
    double dx1 = x - human1_.x;
    double dy1 = y - human1_.y;
    if (dx1 * dx1 + dy1 * dy1 <= human1_.radius * human1_.radius) {
      human1_.hits++;
    }

    // Human 2
    double dx2 = x - human2_.x;
    double dy2 = y - human2_.y;
    if (dx2 * dx2 + dy2 * dy2 <= human2_.radius * human2_.radius) {
      human2_.hits++;
    }
  }

  void addPointToClusters(double x, double y)
  {
    for (auto &c : clusters_) {
      double dx = x - c.cx();
      double dy = y - c.cy();
      double d = std::sqrt(dx * dx + dy * dy);
      if (d < cluster_dist_thresh_) {
        c.sum_x += x;
        c.sum_y += y;
        c.count += 1;
        return;
      }
    }

    Cluster c;
    c.sum_x = x;
    c.sum_y = y;
    c.count = 1;
    clusters_.push_back(c);
  }

  bool clusterNearStaticObstacle(const Cluster &c) const
  {
    if (!have_map_) {
      return true;  // conservative
    }

    int mx, my;
    if (!worldToMap(c.cx(), c.cy(), mx, my)) {
      return true;
    }

    int occupied_cells = 0;
    int window = 3;  // 7x7 window (from -3 to +3)

    for (int dy = -window; dy <= window; ++dy) {
      for (int dx = -window; dx <= window; ++dx) {
        int nx = mx + dx;
        int ny = my + dy;
        if (nx < 0 || ny < 0 ||
            nx >= static_cast<int>(map_.info.width) ||
            ny >= static_cast<int>(map_.info.height)) {
          continue;
        }
        int idx = ny * static_cast<int>(map_.info.width) + nx;
        int8_t occ = map_.data[idx];
        if (occ >= 50) {  // likely static obstacle region
          occupied_cells++;
        }
      }
    }

    // If many cells around centroid are already occupied in the static map,
    // this cluster is probably a wall/shelf, not a new human location.
    return occupied_cells >= 10;
  }

  void resetDetectors()
  {
    clusters_.clear();
    human1_.hits = 0;
    human2_.hits = 0;
  }

  void setupAndNavigate()
  {
    // Wait until we have a map before doing anything
    rclcpp::Rate rate(2.0);
    while (rclcpp::ok() && !have_map_) {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Waiting for /map...");
      rclcpp::spin_some(node_);
      rate.sleep();
    }

    resetDetectors();

    // Set known initial pose (from assignment description)
    auto init = std::make_shared<geometry_msgs::msg::Pose>();
    init->position.x = 2.12;
    init->position.y = -21.3;
    init->position.z = 0.0;
    init->orientation.x = 0.0;
    init->orientation.y = 0.0;
    init->orientation.z = 0.7071;   // yaw ~1.57
    init->orientation.w = 0.7071;

    navigator_.SetInitialPose(init);
    navigator_.WaitUntilNav2Active();

    // Define waypoints to see both humans from multiple angles
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    waypoints.reserve(8);

    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.z = 0.0;
    p.pose.orientation.w = 1.0;

    // Around Human 1
    p.pose.position.x = 2.0;
    p.pose.position.y = -10.0;
    waypoints.push_back(p);

    p.pose.position.x = 1.5;
    p.pose.position.y = -3.0;
    waypoints.push_back(p);

    p.pose.position.x = 1.0;
    p.pose.position.y = -1.0;   // very close to human 1
    waypoints.push_back(p);

    // Transition toward human 2
    p.pose.position.x = 0.0;
    p.pose.position.y = 2.0;
    waypoints.push_back(p);

    p.pose.position.x = -3.0;
    p.pose.position.y = 5.0;
    waypoints.push_back(p);

    p.pose.position.x = -8.0;
    p.pose.position.y = 10.0;
    waypoints.push_back(p);

    // Around Human 2
    p.pose.position.x = -10.0;
    p.pose.position.y = 16.0;   // close to human 2
    waypoints.push_back(p);

    p.pose.position.x = -12.0;
    p.pose.position.y = 15.0;   // another viewpoint of human 2
    waypoints.push_back(p);

    for (auto &wp : waypoints) {
      auto goal = std::make_shared<geometry_msgs::msg::Pose>(wp.pose);
      navigator_.GoToPose(goal);

      while (rclcpp::ok() && !navigator_.IsTaskComplete()) {
        rclcpp::spin_some(node_);
        auto feedback_ptr = navigator_.GetFeedback();
        (void)feedback_ptr;  // could be used for debugging
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      auto result = navigator_.GetResult();
      if (result == rclcpp_action::ResultCode::SUCCEEDED) {
        try {
          // Use latest available transform, no timeout → no dedicated-thread warning
          auto tf = tf_buffer_.lookupTransform(
            "map", "base_link", rclcpp::Time(0));
          RCLCPP_INFO(node_->get_logger(),
                      "Reached waypoint at actual position: x=%.2f, y=%.2f",
                      tf.transform.translation.x,
                      tf.transform.translation.y);
        } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN(node_->get_logger(),
                      "TF lookup at waypoint failed: %s", ex.what());
        }
        RCLCPP_INFO(node_->get_logger(), "Reached waypoint.");
      } else {
        RCLCPP_WARN(node_->get_logger(), "Failed to reach waypoint (result code %d).",
                    static_cast<int>(result));
      }
    }

    reportHumans();
  }

  void reportHumans()
  {
    // Decide if original humans are still present at their old footprints
    bool human1_present = (human1_.hits >= human_seen_threshold_);
    bool human2_present = (human2_.hits >= human_seen_threshold_);

    if (human1_present) {
      RCLCPP_INFO(node_->get_logger(),
                  "Human 1 appears to still be at original location (%.2f, %.2f).",
                  human1_.x, human1_.y);
    } else {
      RCLCPP_INFO(node_->get_logger(),
                  "Human 1 likely moved away from original location (%.2f, %.2f).",
                  human1_.x, human1_.y);
    }

    if (human2_present) {
      RCLCPP_INFO(node_->get_logger(),
                  "Human 2 appears to still be at original location (%.2f, %.2f).",
                  human2_.x, human2_.y);
    } else {
      RCLCPP_INFO(node_->get_logger(),
                  "Human 2 likely moved away from original location (%.2f, %.2f).",
                  human2_.x, human2_.y);
    }

    // If both humans are still at their original locations,
    // we don't need to search for new positions.
    if (human1_present && human2_present) {
      RCLCPP_INFO(node_->get_logger(),
                  "No humans appear to have moved; skipping dynamic obstacle report.");
      return;
    }

    // Filter clusters into "good" dynamic obstacle candidates
    std::vector<Cluster> good;
    for (auto &c : clusters_) {
      if (c.count < min_points_per_cluster_) {
        continue;
      }
      if (clusterNearStaticObstacle(c)) {
        continue;  // likely part of wall/shelf
      }
      good.push_back(c);
    }

    if (good.empty()) {
      RCLCPP_INFO(node_->get_logger(),
                  "No strong dynamic obstacle clusters detected.");
      return;
    }

    // Sort by cluster size descending
    std::sort(good.begin(), good.end(),
              [](const Cluster &a, const Cluster &b) {
                return a.count > b.count;
              });

    // Log top clusters
    size_t max_to_report = std::min(good.size(), static_cast<size_t>(5));
    RCLCPP_INFO(node_->get_logger(),
                "Detected %zu candidate dynamic obstacles (sorted by size).",
                max_to_report);
    for (size_t i = 0; i < max_to_report; ++i) {
      const auto &c = good[i];
      RCLCPP_INFO(node_->get_logger(),
                  "Cluster %zu: center (x=%.2f, y=%.2f), size=%d",
                  i, c.cx(), c.cy(), c.count);
    }

    // Simple nearest-assignment: if a human moved, assign closest cluster
    if (!human1_present && !good.empty()) {
      const Cluster *best = nullptr;
      double best_d2 = std::numeric_limits<double>::max();
      for (auto &c : good) {
        double dx = c.cx() - human1_.x;
        double dy = c.cy() - human1_.y;
        double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
          best_d2 = d2;
          best = &c;
        }
      }
      if (best) {
        RCLCPP_INFO(node_->get_logger(),
                    "Most likely new position for Human 1: (%.2f, %.2f).",
                    best->cx(), best->cy());
      }
    }

    if (!human2_present && !good.empty()) {
      const Cluster *best = nullptr;
      double best_d2 = std::numeric_limits<double>::max();
      for (auto &c : good) {
        double dx = c.cx() - human2_.x;
        double dy = c.cy() - human2_.y;
        double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
          best_d2 = d2;
          best = &c;
        }
      }
      if (best) {
        RCLCPP_INFO(node_->get_logger(),
                    "Most likely new position for Human 2: (%.2f, %.2f).",
                    best->cx(), best->cy());
      }
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto detector = std::make_shared<HumanDetector>();

  rclcpp::spin(detector->getNode());

  rclcpp::shutdown();
  return 0;
}

