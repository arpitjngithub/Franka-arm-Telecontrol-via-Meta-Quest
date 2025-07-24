// vr_continuous_control.cpp
// Continuous Cartesian velocity control for Franka + gripper toggle via VR button.
// Retains only the original debug prints.

#include <array>
#include <algorithm>               // for std::clamp
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <mutex>
#include <thread>
#include <atomic>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>        // for gripper control
#include "examples_common.h"       // setDefaultBehavior()

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp> // for VR controller buttons

// Atomic flag for graceful shutdown
static std::atomic<bool> running(true);
void onSignal(int) { running.store(false); }

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>\n";
    return -1;
  }
  std::signal(SIGINT, onSignal);
  std::string host = argv[1];

  // Initialize ROS2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("franka_vr_control");

  // Shared storage for the latest pose (in millimeters)
  std::mutex mutex;
  geometry_msgs::msg::PoseStamped latest_pose_mm;
  bool received_first = false;

  // Gripper toggle state and debounce
  bool gripper_open = true;
  const int BUTTON_IDX = 0;
  bool was_pressed = false;
  std::mutex grip_mutex;

  // Subscriber debug: print raw mm inputs
  auto subscription = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/vr_cartesian_controller/command", 10,
    [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        latest_pose_mm = *msg;
        received_first = true;
      }
      // original debug print
      std::cout << "[SUB DEBUG] mm: x=" << msg->pose.position.x
                << " y=" << msg->pose.position.y
                << " z=" << msg->pose.position.z << std::endl;
    }
  );

  // Subscriber for gripper toggle button (no extra prints)
  auto joy_sub = node->create_subscription<sensor_msgs::msg::Joy>(
    "/vr/joy", 10,
    [&](const sensor_msgs::msg::Joy::SharedPtr joy) {
      if (joy->buttons.size() > BUTTON_IDX && joy->buttons[BUTTON_IDX]) {
        if (!was_pressed) {
          was_pressed = true;
          std::thread([host, &gripper_open, &grip_mutex]() {
            try {
              franka::Gripper gripper(host);
              std::lock_guard<std::mutex> lk(grip_mutex);
              if (gripper_open) {
                gripper.move(0.1, 0.085);  // open
              } else {
                gripper.move(0.1, 0.0);    // close
              }
              gripper_open = !gripper_open;
            } catch (const std::exception &e) {
              std::cerr << "Gripper exception: " << e.what() << std::endl;
            }
          }).detach();
        }
      } else {
        was_pressed = false;
      }
    }
  );

  // Spin ROS callbacks in background
  std::thread spin_thread([&]() { rclcpp::spin(node); });

  try {
    // Connect to Franka
    franka::Robot robot(host);
    setDefaultBehavior(robot);

    // Debug: initial robot pose
    auto init_state = robot.readOnce();
    std::cout << "[ROBOT DEBUG] start pose (m): x=" << init_state.O_T_EE[12]
              << " y=" << init_state.O_T_EE[13]
              << " z=" << init_state.O_T_EE[14] << std::endl;

    std::cout << "Waiting for first VR pose..." << std::endl;
    while (running.load() && !received_first) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Starting continuous control loop. Press Ctrl-C to stop." << std::endl;

    // Control gains and limits
    const double max_vel = 0.2;    // m/s
    const double Kp      = 0.5;    // proportional gain
    const double max_acc = 0.5;    // m/s^2 for acceleration limiting

    double prev_vx = 0.0, prev_vy = 0.0, prev_vz = 0.0;

    robot.control(
      [&](const franka::RobotState& state, franka::Duration period)
          -> franka::CartesianVelocities {
        double dt = period.toSec();

        // Read latest target (in meters)
        double tx, ty, tz;
        {
          std::lock_guard<std::mutex> lock(mutex);
          tx = latest_pose_mm.pose.position.x / 1000.0;
          ty = latest_pose_mm.pose.position.y / 1000.0;
          tz = latest_pose_mm.pose.position.z / 1000.0;
        }

        // Current end‑effector position
        const auto& O_T_EE = state.O_T_EE;
        double cx = O_T_EE[12], cy = O_T_EE[13], cz = O_T_EE[14];

        // Position error
        double ex = tx - cx;
        double ey = ty - cy;
        double ez = tz - cz;

        // Raw P-controller velocity
        double ux = std::clamp(Kp * ex, -max_vel, max_vel);
        double uy = std::clamp(Kp * ey, -max_vel, max_vel);
        double uz = std::clamp(Kp * ez, -max_vel, max_vel);

        // Acceleration limiting (slew‑rate)
        double max_dv = max_acc * dt;
        double dvx = ux - prev_vx;
        double dvy = uy - prev_vy;
        double dvz = uz - prev_vz;
        if (dvx >  max_dv) ux = prev_vx + max_dv;
        if (dvx < -max_dv) ux = prev_vx - max_dv;
        if (dvy >  max_dv) uy = prev_vy + max_dv;
        if (dvy < -max_dv) uy = prev_vy - max_dv;
        if (dvz >  max_dv) uz = prev_vz + max_dv;
        if (dvz < -max_dv) uz = prev_vz - max_dv;
        prev_vx = ux; prev_vy = uy; prev_vz = uz;

        // Pack into the CartesianVelocities struct
        std::array<double, 6> cart_vel = {ux, uy, uz, 0.0, 0.0, 0.0};
        franka::CartesianVelocities vel(cart_vel);

        // Finish motion if requested
        if (!running.load()) {
          return franka::MotionFinished(vel);
        }
        return vel;
      }
    );

  } catch (const franka::Exception& e) {
    std::cerr << "Franka exception: " << e.what() << std::endl;
    running.store(false);
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}

