cmake_minimum_required(VERSION 3.8)
project(sbamp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros)

# CPP
include_directories(include)

add_executable(sbamp_node src/sbamp_node.cpp)
ament_target_dependencies(sbamp_node 
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2_ros
  )

install(TARGETS
  sbamp_node
  DESTINATION lib/${PROJECT_NAME}
)

# Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/sbamp_node.py
  scripts/visualize_node.py
  scripts/rrt_node.py
  scripts/occupancy_grid_node.py
  scripts/next_waypoint_node.py
  scripts/record_manual_wp_node.py
  scripts/rrt_drive_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Config Files
install(DIRECTORY 
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

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
