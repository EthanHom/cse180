cmake_minimum_required(VERSION 3.8)
project(tb4_human_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(nav2_msgs REQUIRED)
find_package(navigation REQUIRED)

add_executable(human_detector_node
  src/human_detector.cpp)

find_package(visualization_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# add_executable(human_detector_mapdiff src/human_detector_mapdiff_node.cpp)

ament_target_dependencies(human_detector_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  # nav2_msgs
  navigation

  visualization_msgs
  rclcpp_action
  )

install(TARGETS
  human_detector_node
  DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
