#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("robot_state_and_model", node_options);
  const auto& LOGGER = node->get_logger();


  // create the robot model class object, get the kinematic model
  robot_model_loader::RobotModelLoader robot_model_loader(node);
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());

  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
  robot_state->setToDefaultValues();

  // joint model group
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("duman_arm");

  // joint names
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // We can retrieve the current set of joint values stored in the state, this is not updated live
  std::vector<double> joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, joint_values);

  // print the joint values
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // Joint Limits
  // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
  joint_values[0] = 1.57;
  robot_state->setJointGroupPositions(joint_model_group, joint_values);

  // Check whether any joint is outside its joint limits 
  RCLCPP_INFO_STREAM(LOGGER, "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));

  // Enforce the joint limits for this state and check again*/
  robot_state->enforceBounds();
  RCLCPP_INFO_STREAM(LOGGER, "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));

  // Forward Kinematics
  // Now, we can compute forward kinematics for a set of random joint
  // values. Note that we would like to find the pose of the end effector through global link transform
  robot_state->setToRandomPositions(joint_model_group);
  const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform("gripper_base");

  // translation
  RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << end_effector_state.translation() << "\n");

  // Inverse Kinematics

  // rotation converted as euler
  Eigen::Vector3d rpy = end_effector_state.rotation().eulerAngles(2, 1, 0);  // (X=0, Y=1, Z=2)
    std::cout << "Roll: " << rpy[0] 
            << " Pitch: " << rpy[1] 
            << " Yaw: " << rpy[2] << std::endl;

    // define target pose for the Inverse kinematics
    Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity(); // initialize as an identity transform

    // Translation (x, y, z)
    target_pose.translation() << 0.5, 0.0, 0.0;  

    // Orientation as a quaternion
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX());
    target_pose.rotate(q); // means rotate first in z then y then x

  double timeout = 0.1;
  bool found_ik = robot_state->setFromIK(joint_model_group, target_pose, timeout);

  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(LOGGER, "bon kamanna halli Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Did not find IK solution");
  }

  // Get the Jacobian
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                           reference_point_position, jacobian);
  RCLCPP_INFO_STREAM(LOGGER, "Jacobian: \n" << jacobian << "\n");

  rclcpp::shutdown();
  return 0;
}