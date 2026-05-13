import os
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Arguments
    target_color = LaunchConfiguration("target_color")
    is_ignition = LaunchConfiguration("is_ignition")

    target_color_arg = DeclareLaunchArgument(
        "target_color", default_value="B",
        description="Target color to pick (R, G, B)"
    )

    is_ignition_arg = DeclareLaunchArgument(
        "is_ignition", default_value="true",
        description="Use Ignition Gazebo if true"
    )

    # Build MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="panda_moveit")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("panda_description"),
                "urdf",
                "panda.urdf.xacro"
            ),
            mappings={"is_ignition": is_ignition}
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    pick_and_place_node = Node(
        package="panda_moveit",
        executable="pick_and_place",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"target_color": target_color},
            {"use_sim_time": True}
        ]
    )

    return LaunchDescription([
        target_color_arg,
        is_ignition_arg,
        pick_and_place_node
    ])
