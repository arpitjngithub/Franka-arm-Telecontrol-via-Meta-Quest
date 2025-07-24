// cartesian_pose_listener.cpp

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class CartesianPoseListener : public rclcpp::Node {
public:
  CartesianPoseListener()
  : Node("cartesian_pose_listener")
  {
    // Subscribe only to the right‐controller pose
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vr/controller_right/pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void {
        this->process_pose(msg);
      });

    // Publisher to the Franka Cartesian controller command topic
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/vr_cartesian_controller/command", 10);
  }

private:
  void process_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // 1) convert VR meters → millimeters
    double vr_x_mm = msg->pose.position.x * 1000.0;
    double vr_y_mm = msg->pose.position.y * 1000.0;
    double vr_z_mm = msg->pose.position.z * 1000.0;

    // 2) axis mapping & bias:
    //    VR +Z → FR +X
    //    VR +X → FR –Y
    //    VR +Y → FR +Z
    //
    //    Biases were chosen so that
    //      VR(-33.961, 1236.663, 216.206) → FR(307, 0.3, 486.7)
    constexpr double bias_x =  90.794;   // FR_x bias
    constexpr double bias_y = -33.661;   // FR_y bias
    constexpr double bias_z = -749.963;  // FR_z bias

    double fr_x = vr_z_mm + bias_x;
    double fr_y = -vr_x_mm + bias_y;
    double fr_z = vr_y_mm + bias_z;

    RCLCPP_INFO(this->get_logger(),
      "Streaming to Franka: x=%.3f y=%.3f z=%.3f",
      fr_x, fr_y, fr_z);

    // 3) publish to Franka controller
    geometry_msgs::msg::PoseStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "fr3_link0";  // match your robot’s base frame

    cmd.pose.position.x = fr_x;
    cmd.pose.position.y = fr_y;
    cmd.pose.position.z = fr_z;
    // leave orientation unchanged (or remap if needed)
    cmd.pose.orientation = msg->pose.orientation;

    publisher_->publish(cmd);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianPoseListener>());
  rclcpp::shutdown();
  return 0;
}

