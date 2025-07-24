cmake_minimum_required(VERSION 3.5)
project(franka_vr_control)

# Require C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- ament and ROS2 dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs   REQUIRED)

# --- Core Franka driver ---
find_package(Franka 0.13.2 REQUIRED)

# --- Eigen for examples_common ---
find_package(Eigen3 REQUIRED)

# --- Bring in examples_common from libfranka/examples ---
# Assumes libfranka/examples is at ../libfranka/examples
set(LIBFRANKA_EXAMPLES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../libfranka/examples")
add_library(examples_common STATIC
  "${LIBFRANKA_EXAMPLES_DIR}/examples_common.cpp"
)
target_include_directories(examples_common PUBLIC
  "${LIBFRANKA_EXAMPLES_DIR}"
)
# Link Eigen and Franka include/libs for examples_common
target_link_libraries(examples_common PUBLIC
  Eigen3::Eigen
  Franka::Franka
)

# --- Continuous VR→Franka control node ---
add_executable(vr_cartesian_control src/vr_cartesian_control.cpp)
ament_target_dependencies(vr_cartesian_control
  rclcpp
  geometry_msgs
  sensor_msgs
)
target_link_libraries(vr_cartesian_control
  examples_common
)

# --- Install the executable ---
install(TARGETS vr_cartesian_control
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

