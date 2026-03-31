cmake_minimum_required(VERSION 3.8)
project(cs4mt_armfootprint_aware_navigation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(cs4home_core REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
  rclcpp_action
  rclcpp_components
  behaviortree_cpp_v3
  cs4home_core
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  visualization_msgs
  lifecycle_msgs
  nav2_msgs
)

# ===========================================================================
# cs4mt_armfootprint_aware_navigation
# Real-time MPO-700 + UR10 footprint projection onto Nav2 costmaps
# ===========================================================================

# CORE — TF projection + convex hull + set_parameters
add_library(arm_footprint_cognitive_core SHARED
  src/MTArmFootprintCognitiveCore.cpp)
ament_target_dependencies(arm_footprint_cognitive_core ${dependencies})
target_link_libraries(arm_footprint_cognitive_core)

# AFFERENT — subscribes to /joint_states
add_library(arm_footprint_input SHARED
  src/MTArmFootprintInput.cpp)
ament_target_dependencies(arm_footprint_input ${dependencies})
target_link_libraries(arm_footprint_input)

# EFFERENT — publishes to /local_costmap/footprint, /global_costmap/footprint
add_library(arm_footprint_output SHARED
  src/MTArmFootprintOutput.cpp)
ament_target_dependencies(arm_footprint_output ${dependencies})
target_link_libraries(arm_footprint_output)

# MODULE executable
add_executable(arm_footprint_cognitive_module
  src/MTArmFootprintCognitiveModule.cpp)
ament_target_dependencies(arm_footprint_cognitive_module ${dependencies})

# ===========================================================================
# INSTALLATION
# ===========================================================================

# Component libraries (loaded dynamically by cs4home_core)
install(TARGETS
  arm_footprint_cognitive_core
  arm_footprint_input
  arm_footprint_output
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Module executable
install(TARGETS
  arm_footprint_cognitive_module
  DESTINATION lib/${PROJECT_NAME}
)

# Config and launch directories
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
