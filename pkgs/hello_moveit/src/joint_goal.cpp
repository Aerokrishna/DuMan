#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_group_joint_goal_demo");

  // 1️⃣ Create MoveGroupInterface for your robot arm
  static const std::string PLANNING_GROUP = "duman_arm";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // Optional: enable replanning if initial plan fails
  move_group.allowReplanning(true);

  // 2️⃣ Set a joint goal
  std::vector<double> joint_goal = { -1.0, 1.7, 0.7, -1.5, 0.0, 0.3}; // your 5-DOF joint values
  move_group.setJointValueTarget(joint_goal);

  // // 3️⃣ Add a custom constraint (orientation constraint)
  // moveit_msgs::msg::OrientationConstraint ocm;
  // ocm.link_name = "gripper_base";               // end-effector link
  // ocm.header.frame_id = "base";            // reference frame
  // ocm.orientation.w = 1.0;                 // desired orientation
  // ocm.absolute_x_axis_tolerance = 0.1;     // radians
  // ocm.absolute_y_axis_tolerance = 0.1;
  // ocm.absolute_z_axis_tolerance = 0.1;
  // ocm.weight = 1.0;

  // moveit_msgs::msg::Constraints path_constraints;
  // path_constraints.orientation_constraints.push_back(ocm);
  // move_group.setPathConstraints(path_constraints);

  // 4️⃣ Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(node->get_logger(), "brr brr Planning successful. Executing...");
    // 5️⃣ Execute the plan
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed!");
  }

  rclcpp::shutdown();
  return 0;
}
