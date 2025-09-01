#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_group_position_or_cartesian_demo");

    // Initialize MoveGroupInterface
    static const std::string PLANNING_GROUP = "duman_arm";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.allowReplanning(true);

    // // Get the current state as a starting point
    // moveit::core::RobotState start_state(*move_group.getCurrentState());

    // // Set all joints to their default positions
    // start_state.setToDefaultValues();

    // // Apply the start state to MoveGroup
    // move_group.setStartState(start_state);

    //  Define target position
    geometry_msgs::msg::Point target_point;
    target_point.x = 0.34;
    target_point.y = 0.0;
    target_point.z = 0.0;

    //  Position-only constraint
    moveit_msgs::msg::PositionConstraint pcm;
    pcm.link_name = "end_effector";       // end-effector link
    pcm.header.frame_id = "base";         // reference frame

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {0.01, 0.01, 0.01};  // 1 cm³ tolerance

    geometry_msgs::msg::Pose box_pose;
    box_pose.position = target_point;
    box_pose.orientation.w = 1.0;

    pcm.constraint_region.primitives.push_back(box);
    pcm.constraint_region.primitive_poses.push_back(box_pose);
    pcm.weight = 1.0;

    moveit_msgs::msg::Constraints goal_constraints;
    goal_constraints.position_constraints.push_back(pcm);

    move_group.setPathConstraints(goal_constraints);

    // Try OMPL plan with constraints
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(node->get_logger(), "Planning with position-only constraints successful. Executing...");
        move_group.execute(plan);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Planning with constraints failed. Trying Cartesian path...");

        //Cartesian path fallback
        std::vector<geometry_msgs::msg::Pose> waypoints;
        for (int i=0; i<5; i++){waypoints.push_back(move_group.getCurrentPose("end_effector").pose);}
          // start
        geometry_msgs::msg::Pose target_pose = waypoints.back();
        target_pose.position = target_point;                    // update position
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group.computeCartesianPath(
            waypoints,    // waypoints
            0.01,         // eef_step (1 cm resolution)
            0.0,          // jump_threshold (disabled)
            trajectory
        );

        if (fraction > 0.9)
        {
            RCLCPP_INFO(node->get_logger(), "Cartesian path achieved %.2f%% of waypoints. Executing...", fraction * 100.0);
            move_group.execute(trajectory);
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Cartesian path failed. Only %.2f%% achieved", fraction * 100.0);
        }
    }

    rclcpp::shutdown();
    return 0;
}
