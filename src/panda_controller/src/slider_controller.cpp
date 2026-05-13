#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class SliderControl : public rclcpp::Node
{
public:
  SliderControl() : Node("slider_control")
  {
    arm_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "arm_controller/joint_trajectory", 10);
    gripper_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "gripper_controller/joint_trajectory", 10);
      
    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_commands", 10, std::bind(&SliderControl::sliderCallback, this, std::placeholders::_1));
      
    RCLCPP_INFO(this->get_logger(), "Slider Control Node started");
  }

private:
  void sliderCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() < 8) {
      RCLCPP_WARN(this->get_logger(), "Expected at least 8 joint positions");
      return;
    }

    auto arm_controller = trajectory_msgs::msg::JointTrajectory();
    auto gripper_controller = trajectory_msgs::msg::JointTrajectory();

    arm_controller.joint_names = {
      "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
      "panda_joint5", "panda_joint6", "panda_joint7"
    };
    gripper_controller.joint_names = {"panda_finger_joint1"};

    trajectory_msgs::msg::JointTrajectoryPoint arm_goal;
    trajectory_msgs::msg::JointTrajectoryPoint gripper_goal;

    for (size_t i = 0; i < 7; ++i) {
      arm_goal.positions.push_back(msg->position[i]);
    }
    gripper_goal.positions.push_back(msg->position[7]);

    arm_controller.points.push_back(arm_goal);
    gripper_controller.points.push_back(gripper_goal);

    arm_pub_->publish(arm_controller);
    gripper_pub_->publish(gripper_controller);
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SliderControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
