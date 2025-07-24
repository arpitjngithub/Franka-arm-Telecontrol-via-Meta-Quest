#include "vr_franka_interface/vr_cartesian_controller.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace vr_franka_interface {

//–– constructor ––
VrCartesianController::VrCartesianController() = default;

//–– declare and initialize parameters ––
controller_interface::CallbackReturn VrCartesianController::on_init() {
  auto node = get_node();
  node->declare_parameter<std::string>("arm_id", "");
  node->declare_parameter<std::vector<std::string>>(
    "command_interface_names", {"cartesian_pose_command"});
  node->declare_parameter<std::vector<std::string>>(
    "state_interface_names", {"O_T_EE"});
  node->declare_parameter<int>("action_monitor_rate", 1000);
  node->declare_parameter<std::vector<double>>(
    "lower_joint_torque_thresholds", std::vector<double>(7, 0.0));
  node->declare_parameter<std::vector<double>>(
    "upper_joint_torque_thresholds", std::vector<double>(7, 0.0));
  node->declare_parameter<std::vector<double>>(
    "lower_cartesian_force_thresholds", std::vector<double>(6, 0.0));
  node->declare_parameter<std::vector<double>>(
    "upper_cartesian_force_thresholds", std::vector<double>(6, 0.0));
  return CallbackReturn::SUCCESS;
}

//–– which command interfaces we need ––
controller_interface::InterfaceConfiguration
VrCartesianController::command_interface_configuration() const {
  auto node = get_node();
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    node->get_parameter("command_interface_names").as_string_array()
  };
}

//–– which state interfaces we read ––
controller_interface::InterfaceConfiguration
VrCartesianController::state_interface_configuration() const {
  auto node = get_node();
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    node->get_parameter("state_interface_names").as_string_array()
  };
}

//–– subscribe to your VR-topic ––
controller_interface::CallbackReturn
VrCartesianController::on_configure(const rclcpp_lifecycle::State &) {
  auto node = get_node();
  node->get_parameter("arm_id", arm_id_);
  subscription_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/vr_cartesian_controller/command",
    rclcpp::QoS(1),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      // build a 4×4 row-major flat array
      std::array<double,16> M{{ 
        1,0,0, msg->pose.position.x,
        0,1,0, msg->pose.position.y,
        0,0,1, msg->pose.position.z,
        0,0,0, 1
      }};
      buffer_.writeFromNonRT(M);
    });
  return CallbackReturn::SUCCESS;
}

//–– seed the buffer with the current EE pose ––
controller_interface::CallbackReturn
VrCartesianController::on_activate(const rclcpp_lifecycle::State &) {
  std::array<double,16> init{};
  for (size_t i = 0; i < init.size(); ++i) {
    init[i] = state_interfaces_[i].get_value();
  }
  buffer_.writeFromNonRT(init);
  return CallbackReturn::SUCCESS;
}

//–– on each control loop ––
controller_interface::return_type
VrCartesianController::update(
  const rclcpp::Time &, const rclcpp::Duration &) 
{
  // read the last-written target
  auto target_ptr = buffer_.readFromRT();
  if (target_ptr) {
    auto const & target = *target_ptr;
    for (size_t i = 0; i < target.size(); ++i) {
      command_interfaces_[i].set_value(target[i]);
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace vr_franka_interface

