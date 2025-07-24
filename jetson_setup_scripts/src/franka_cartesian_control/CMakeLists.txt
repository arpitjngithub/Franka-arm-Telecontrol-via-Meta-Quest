cmake_minimum_required(VERSION 3.8)
project(franka_cartesian_control)

# Use C++14 or higher standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(franka_semantic_components REQUIRED)  # Adding the dependency for 'franka_semantic_components'

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${franka_semantic_components_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR} # Include franka_semantic_components headers
)

# Create the executable
add_executable(cartesian_pose_listener src/cartesian_pose_listener.cpp)

# Link against the necessary libraries
ament_target_dependencies(cartesian_pose_listener
  rclcpp
  geometry_msgs
  franka_semantic_components  # Link with franka_semantic_components
)

# Install the executable
install(TARGETS
  cartesian_pose_listener
  DESTINATION lib/${PROJECT_NAME}
)

# Install dependencies
ament_package()

