"""Bring up the physical UR5e, MoveIt, D435i, task nodes, FlexBE, and one RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


MOVEIT_PACKAGE = "ur5e_mpm_d435i_rpi5_millibar_r2f85_moveit_config"


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    headless = LaunchConfiguration("headless")
    launch_rviz = LaunchConfiguration("launch_rviz")

    armada_launch = PathJoinSubstitution(
        [FindPackageShare("armada_bringup"), "launch"]
    )
    moveit_launch = PathJoinSubstitution(
        [FindPackageShare(MOVEIT_PACKAGE), "launch", "moveit.launch.py"]
    )

    robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [armada_launch, "modules", "arms", "ur5e", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "ur_type": "ur5e",
            "mount": "mpm",
            "sensor": "d435i",
            "controller": "rpi5",
            "tool_change_hardware": "millibar",
            "end_effector": "robotiq_2f85",
            "tf_prefix": "ur5e_",
            "launch_rviz": "false",
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        launch_arguments={
            "ur_type": "ur5e",
            "launch_rviz": launch_rviz,
            "rviz_config_file": PathJoinSubstitution(
                [FindPackageShare(MOVEIT_PACKAGE), "config", "moveit.rviz"]
            ),
        }.items(),
    )

    flexbe_and_task_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([armada_launch, "flexbe_robot_bringup.launch.py"])
        ),
        launch_arguments={
            "headless": headless,
            "launch_move_group": "false",
            "launch_realsense": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                default_value="10.10.10.152",
                description="IP address of the physical UR controller.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="Run FlexBE without its web UI frontend.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch the single MoveIt-configured RViz instance.",
            ),
            robot_control,
            moveit,
            flexbe_and_task_nodes,
        ]
    )
