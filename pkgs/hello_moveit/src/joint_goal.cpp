
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true) // needed for moveit
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "duman_arm");

  // Get the joint model group
  // Wait until the CurrentStateMonitor gets a valid robot state
    rclcpp::sleep_for(std::chrono::seconds(2));  // give joint_state_publisher time
    move_group_interface.setStartStateToCurrentState();

    auto joint_model_group =
    move_group_interface.getCurrentState()->getJointModelGroup("duman_arm");

    // Create a vector of joint positions
    std::vector<double> joint_group_positions = {
    1.57,   // joint1
    0.0, // joint2
    0.0,   // joint3
    0.0,   // joint4
    0.0    // joint5
    };

    // Set as the target
    move_group_interface.setJointValueTarget(joint_group_positions);

    // Plan & execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if(success)
    move_group_interface.execute(plan);
    else
    RCLCPP_ERROR(logger, "Joint goal planning failed!");


  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}