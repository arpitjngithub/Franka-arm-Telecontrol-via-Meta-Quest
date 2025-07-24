cmake_minimum_required(VERSION 3.5)
project(vr_franka_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

# 1) Listener node to forward VR poses into a topic
add_executable(cartesian_pose_listener src/cartesian_pose_listener.cpp)
ament_target_dependencies(cartesian_pose_listener
  rclcpp
  geometry_msgs
)
install(TARGETS cartesian_pose_listener
  DESTINATION lib/${PROJECT_NAME}
)

# 2) The VR → Franka Cartesian controller plugin
add_library(vr_cartesian_controller SHARED src/vr_cartesian_controller.cpp)
target_include_directories(vr_cartesian_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(vr_cartesian_controller
  rclcpp
  rclcpp_lifecycle
  controller_interface
  pluginlib
  realtime_tools
  hardware_interface
  geometry_msgs
)

# Expose our plugin.xml so pluginlib can find it
pluginlib_export_plugin_description_file(
  vr_franka_interface
  plugin_description.xml
)

install(TARGETS vr_cartesian_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # your executable can stay here
)


install(DIRECTORY include/vr_franka_interface
  DESTINATION include
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

