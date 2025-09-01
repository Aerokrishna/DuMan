from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("onshape", package_name="duman_moveit_config")
        .robot_description(file_path="config/onshape.urdf.xacro")
        .robot_description_semantic(file_path="config/onshape.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            # Node(
            # package="moveit_ros_move_group",
            # executable="move_group",
            # output="screen",
            # parameters=[
            #     moveit_config.to_dict(),
            #     {"planning_plugin": ["ompl_interface/OMPLPlanner"]},
            # ]),
            Node(
                package="hello_moveit",
                executable="pose_goal",
                name="pose_goal",
                output="screen",
                parameters=[
                moveit_config.to_dict(),
                {"planning_plugin": ["ompl_interface/OMPLPlanner"]},
            ],
            )
        ]
    )
