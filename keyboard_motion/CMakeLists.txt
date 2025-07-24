cmake_minimum_required(VERSION 3.15)
project(my_second_motion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(Franka_DIR "/opt/ros/humble/lib/x86_64-linux-gnu/cmake/Franka")

find_package(Franka REQUIRED)
find_package(fmt REQUIRED)


add_executable(my_second_motion my_second_motion.cpp)

target_link_libraries(my_second_motion PRIVATE Franka::Franka fmt::fmt)


