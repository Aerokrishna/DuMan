from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("onshape", package_name="duman_moveit_config").to_moveit_configs()

    tutorial_node = Node(
        package="hello_moveit",
        executable="robot_state_and_model",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([tutorial_node])