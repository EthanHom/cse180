// Terminal Commands
/*

Terminal Commands

Terminal 1:
(.venv) root@docker-desktop:/MRTP/MRTP# source install/local_setup.bash 
(.venv) root@docker-desktop:/MRTP/MRTP# source install/setup.bash 
(.venv) root@docker-desktop:/MRTP/MRTP# ros2 launch gazeboenvs tb3_simulation.launch.py headless:=False

Terminal 2: 
(.venv) root@docker-desktop:/MRTP/lab7# source /MRTP/MRTP/install/setup.bash
(.venv) root@docker-desktop:/MRTP/lab7# source /MRTP/lab7/install/setup.bash
(.venv) root@docker-desktop:/MRTP/lab7# colcon build
(.venv) root@docker-desktop:/MRTP/lab7# ros2 run lab7 testnavigator_lab7

*/

// Original Code from Claude Sonnet 4.5 Thinking
/* 

#include <rclcpp/rclcpp.hpp>
#include <navigation/navigation.hpp>
#include <iostream>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char* argv[]) {
    // Initialize ROS
    rclcpp::init(argc, argv);
    
    // Create Navigator instance with debug info
    Navigator navigator(true);
    
    // Set initial pose (mandatory before any navigation calls)
    geometry_msgs::msg::Pose::SharedPtr init = 
        std::make_shared<geometry_msgs::msg::Pose>();
    init->position.x = -2.0;
    init->position.y = -0.5;
    init->position.z = 0.0;
    init->orientation.w = 1.0;
    
    navigator.SetInitialPose(init);
    std::cout << "Initial pose set" << std::endl;
    
    // Wait for Nav2 to become active
    navigator.WaitUntilNav2Active();
    std::cout << "Nav2 is active" << std::endl;
    
    // Create a goal pose
    geometry_msgs::msg::Pose::SharedPtr goal_pos = 
        std::make_shared<geometry_msgs::msg::Pose>();
    goal_pos->position.x = 2.0;
    goal_pos->position.y = 1.0;
    goal_pos->position.z = 0.0;
    goal_pos->orientation.w = 1.0;
    
    std::cout << "\nCalling GetPath to goal position (2.0, 1.0)..." << std::endl;
    
    // Call GetPath to compute a path
    // GetPath is SYNCHRONOUS - it returns immediately with the result
    auto path = navigator.GetPath(goal_pos);
    
    // Check if path was successfully computed by checking if pointer is null
    if (path == nullptr) {
        std::cout << "\n=== GetPath FAILED ===" << std::endl;
        std::cout << "No path could be found to the goal." << std::endl;
    } else {
        std::cout << "\n=== GetPath SUCCEEDED ===" << std::endl;
        std::cout << "\nPath contains " << path->poses.size() 
                  << " waypoints:\n" << std::endl;
        
        // Iterate through all poses in the path
        for (size_t i = 0; i < path->poses.size(); ++i) {
            const auto& pose_stamped = path->poses[i];
            
            // Extract position (x, y)
            double x = pose_stamped.pose.position.x;
            double y = pose_stamped.pose.position.y;
            
            // Convert quaternion to yaw angle
            tf2::Quaternion q(
                pose_stamped.pose.orientation.x,
                pose_stamped.pose.orientation.y,
                pose_stamped.pose.orientation.z,
                pose_stamped.pose.orientation.w
            );
            
            // Extract yaw from quaternion
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            // Print waypoint information
            std::cout << "Waypoint " << i << ": "
                      << "x = " << x << ", "
                      << "y = " << y << ", "
                      << "yaw = " << yaw << " rad" << std::endl;
        }
    }
    
    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}

*/


// Code from navigation.hpp with Claude Sonnet 4.5 Thinking starting at line 164
#include <rclcpp/rclcpp.hpp> 
#include <navigation/navigation.hpp>
#include <iostream>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>



int main(int argc,char **argv) {
 
  rclcpp::init(argc,argv); // initialize ROS 
  Navigator navigator(true,false); // create node with debug info but not verbose

  // first: it is mandatory to initialize the pose of the robot
  geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
  init->position.x = -2;
  init->position.y = -0.5;
  init->orientation.w = 1;
  navigator.SetInitialPose(init);
  // wait for navigation stack to become operationale
  navigator.WaitUntilNav2Active();
  // spin in place of 90 degrees (default parameter)
  navigator.Spin();
  while ( ! navigator.IsTaskComplete() ) {
    // busy waiting for task to be completed
  }
  geometry_msgs::msg::Pose::SharedPtr goal_pos = std::make_shared<geometry_msgs::msg::Pose>();
  goal_pos->position.x = 2;
  goal_pos->position.y = 1;
  goal_pos->orientation.w = 1;
  // move to new pose
  navigator.GoToPose(goal_pos);
  while ( ! navigator.IsTaskComplete() ) {
    
  }
  goal_pos->position.x = 2;
  goal_pos->position.y = -1;
  goal_pos->orientation.w = 1;
  navigator.GoToPose(goal_pos);
  // move to new pose
  while ( ! navigator.IsTaskComplete() ) {
    
  }
  // backup of 0.15 m (deafult distance)
  navigator.Backup();
  while ( ! navigator.IsTaskComplete() ) {
    
  }

  // complete here....

    // std::cout << "\nCalling GetPath to goal position (2.0, 1.0)..." << std::endl;
    std::cout << "\nCalling GetPath to goal position (2.0, -1.0)..." << std::endl;
    
    // Call GetPath to compute a path
    // GetPath is SYNCHRONOUS - it returns immediately with the result
    auto path = navigator.GetPath(goal_pos);
    
    // Check if path was successfully computed by checking if pointer is null

    if (path == nullptr || path->poses.empty()) {
        std::cout << "\n=== GetPath FAILED ===" << std::endl;
        std::cout << "No path could be found to the goal." << std::endl;
    } else {
        std::cout << "\n=== GetPath SUCCEEDED ===" << std::endl;
        std::cout << "\nPath contains " << path->poses.size() 
                  << " waypoints:\n" << std::endl;
        
        // Iterate through all poses in the path
        for (size_t i = 0; i < path->poses.size(); ++i) {
            const auto& pose_stamped = path->poses[i];
            
            // Extract position (x, y)
            double x = pose_stamped.pose.position.x;
            double y = pose_stamped.pose.position.y;
            
            // Convert quaternion to yaw angle
            tf2::Quaternion q(
                pose_stamped.pose.orientation.x,
                pose_stamped.pose.orientation.y,
                pose_stamped.pose.orientation.z,
                pose_stamped.pose.orientation.w
            );
            
            // Extract yaw from quaternion
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            // Print waypoint information
            std::cout << "Waypoint " << i << ": "
                      << "x = " << x << ", "
                      << "y = " << y << ", "
                      << "yaw = " << yaw << " rad" << std::endl;
        }
    }

  
  rclcpp::shutdown(); // shutdown ROS
  return 0;
}
