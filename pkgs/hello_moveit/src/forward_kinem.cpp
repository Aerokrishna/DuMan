#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("random_fk_goal_node");
  const auto& LOGGER = node->get_logger();

  // Load robot model
  robot_model_loader::RobotModelLoader robot_model_loader(node);
  auto kinematic_model = robot_model_loader.getModel();
  RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());

  // Robot state
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
  robot_state->setToDefaultValues();

  // Joint group
  const auto* joint_model_group = kinematic_model->getJointModelGroup("duman_arm");
  const auto& joint_names = joint_model_group->getVariableNames();

  // Sample random joint positions
  robot_state->setToRandomPositions(joint_model_group);
  robot_state->enforceBounds();

  // Copy the joint values
  std::vector<double> joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, joint_values);

  // Compute FK using getPositionFK interface
  std::vector<std::string> links = { "gripper_base" };  // EE link
  std::vector<geometry_msgs::msg::Pose> poses;

  bool success = robot_state->getRobotModel()->getJointModelGroup("duman_arm")->getSolverInstance()
      ->getPositionFK(links, joint_values, poses);

  if (success)
  {
    RCLCPP_INFO_STREAM(LOGGER, "Randomly sampled FK pose for EE: ");
    RCLCPP_INFO_STREAM(LOGGER, "Position: x=" << poses[0].position.x << ", y=" << poses[0].position.y
                                               << ", z=" << poses[0].position.z);
    RCLCPP_INFO_STREAM(LOGGER, "Orientation: x=" << poses[0].orientation.x << ", y=" << poses[0].orientation.y
                                                  << ", z=" << poses[0].orientation.z
                                                  << ", w=" << poses[0].orientation.w);
  }
  else
  {
    RCLCPP_WARN(LOGGER, "FK solution not found!");
  }

  rclcpp::shutdown();
  return 0;
}
