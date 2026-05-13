#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <thread>

class PickAndPlace : public rclcpp::Node
{
public:
  PickAndPlace(const rclcpp::NodeOptions & options)
  : Node("pick_and_place", options), already_moved_(false)
  {
    this->declare_parameter("target_color", "R");
    target_color_ = this->get_parameter("target_color").as_string();
    
    this->declare_parameter("approach_offset", 0.31);
    approach_offset_ = this->get_parameter("approach_offset").as_double();

    // Callback group for MoveIt
    moveit_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = sub_cb_group_;
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/color_coordinates", 10,
      std::bind(&PickAndPlace::coords_callback, this, std::placeholders::_1),
      sub_opt);

    RCLCPP_INFO(this->get_logger(), "Waiting for %s from /color_coordinates...", target_color_.c_str());

    // Joint positions
    start_joints_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, deg2rad(-125.0)};
    home_joints_ = {0.0, 0.0, 0.0, deg2rad(-90.0), 0.0, deg2rad(92.0), deg2rad(50.0)};
    drop_joints_ = {deg2rad(-155.0), deg2rad(30.0), deg2rad(-20.0),
                    deg2rad(-124.0), deg2rad(44.0), deg2rad(163.0), deg2rad(7.0)};
  }

  void initializeMoveGroup()
  {
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("pick_and_place_moveit", node_options);
    
    std::thread([this, move_group_node]() {
      arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, "arm");
      gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, "gripper");

      arm_group_->setEndEffectorLink("panda_hand");

      arm_group_->setMaxVelocityScalingFactor(0.1);
      arm_group_->setMaxAccelerationScalingFactor(0.1);

      RCLCPP_INFO(this->get_logger(), "Moving to start configuration...");
      arm_group_->setJointValueTarget(start_joints_);
      arm_group_->move();
      RCLCPP_INFO(this->get_logger(), "Ready.");
    }).detach();
  }

private:
  double deg2rad(double deg) { return deg * M_PI / 180.0; }

  void operate_gripper(bool open)
  {
    if (!gripper_group_) return;
    std::vector<double> joint_values = open ? std::vector<double>{0.04, 0.04} : std::vector<double>{0.0, 0.0};
    gripper_group_->setJointValueTarget(joint_values);
    gripper_group_->move();
  }

  void coords_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (already_moved_) return;

    std::string data = msg->data;
    size_t pos1 = data.find(',');
    size_t pos2 = data.find(',', pos1 + 1);
    size_t pos3 = data.find(',', pos2 + 1);

    if (pos1 == std::string::npos || pos2 == std::string::npos || pos3 == std::string::npos) {
      return;
    }

    std::string color_id = data.substr(0, pos1);
    
    if (color_id == target_color_ && arm_group_) {
      already_moved_ = true;
      double x = std::stod(data.substr(pos1 + 1, pos2 - pos1 - 1));
      double y = std::stod(data.substr(pos2 + 1, pos3 - pos2 - 1));
      double z = std::stod(data.substr(pos3 + 1));

      RCLCPP_INFO(this->get_logger(), "Target %s locked at: [%.3f, %.3f, %.3f]", target_color_.c_str(), x, y, z);

      geometry_msgs::msg::Pose pick_pose;
      pick_pose.position.x = x;
      pick_pose.position.y = y;
      pick_pose.position.z = z - 0.60;
      pick_pose.orientation.x = 0.0;
      pick_pose.orientation.y = 1.0;
      pick_pose.orientation.z = 0.0;
      pick_pose.orientation.w = 0.0;

      // 1. Home
      arm_group_->setJointValueTarget(home_joints_);
      arm_group_->move();

      // 2. Move above target
      arm_group_->setPoseTarget(pick_pose);
      arm_group_->move();

      // 3. Open gripper
      operate_gripper(true);

      // 4. Approach
      geometry_msgs::msg::Pose approach_pose = pick_pose;
      approach_pose.position.z -= approach_offset_;
      
      // Compute cartesian path for approach
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(approach_pose);
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      arm_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      arm_group_->execute(trajectory);

      // 5. Close gripper
      operate_gripper(false);

      // 6. Move to home joint configuration
      arm_group_->setJointValueTarget(home_joints_);
      arm_group_->move();

      // 7. Move to drop joint configuration
      arm_group_->setJointValueTarget(drop_joints_);
      arm_group_->move();

      // 8. Open gripper
      operate_gripper(true);

      // 9. Close gripper
      operate_gripper(false);

      // 10. Start configuration
      arm_group_->setJointValueTarget(start_joints_);
      arm_group_->move();

      RCLCPP_INFO(this->get_logger(), "Pick-and-place sequence complete.");
      rclcpp::shutdown();
    }
  }

  std::string target_color_;
  double approach_offset_;
  bool already_moved_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::CallbackGroup::SharedPtr moveit_cb_group_;
  rclcpp::CallbackGroup::SharedPtr sub_cb_group_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

  std::vector<double> start_joints_;
  std::vector<double> home_joints_;
  std::vector<double> drop_joints_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<PickAndPlace>(node_options);
  
  // Need multithreaded executor for MoveGroup and subscriptions
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  node->initializeMoveGroup();
  
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
