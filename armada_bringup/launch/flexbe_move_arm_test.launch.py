import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


MOVEIT_CONFIG_PACKAGE = (
    "ur5e_mpm_d435i_rpi5_millibar_r2f85_moveit_config"
)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    launch_flexbe = LaunchConfiguration("launch_flexbe")

    # The MoveIt configuration's .setup_assistant file supplies the xacro
    # mappings for the ur5e/mpm/d435i/rpi5/millibar/robotiq_2f85 robot.
    moveit_config = (
        MoveItConfigsBuilder(
            "armada_robot",
            package_name=MOVEIT_CONFIG_PACKAGE,
        )
        .to_moveit_configs()
    )

    robot_description = moveit_config.robot_description
    robot_description_semantic = moveit_config.robot_description_semantic
    robot_description_kinematics = moveit_config.robot_description_kinematics
    ompl_planning_pipeline_config = moveit_config.planning_pipelines
    planning_scene_monitor_parameters = moveit_config.planning_scene_monitor
    joint_limits_yaml = moveit_config.joint_limits

    # Keep execution settings and controller configuration distinct so the
    # move_group parameter list mirrors the original launch file.
    trajectory_execution = {
        key: value
        for key, value in moveit_config.trajectory_execution.items()
        if key in ("moveit_manage_controllers", "trajectory_execution")
    }
    moveit_controllers = {
        key: value
        for key, value in moveit_config.trajectory_execution.items()
        if key in (
            "moveit_controller_manager",
            "moveit_simple_controller_manager",
        )
    }

    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability",
    }

    move_named = Node(
        package="move_group_ros2",
        executable="move_to_named_pose_service",
        name="move_to_named_pose_service",
        output="screen",
        parameters=[
            {"planning_group": "arm"},
            robot_description,
            robot_description_semantic,
        ],
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        emulate_tty=True,
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            {"use_sim_time": use_sim_time},
            move_group_capabilities,
        ],
    )

    flexbe_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("flexbe_webui"),
                "launch",
                "flexbe_full.launch.py",
            )
        ),
        launch_arguments={"headless": headless}.items(),
        condition=IfCondition(launch_flexbe),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock instead of wall clock.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run FlexBE without the web UI frontend.",
            ),
            DeclareLaunchArgument(
                "launch_flexbe",
                default_value="true",
                description="Start FlexBE onboard and operator control system.",
            ),
            flexbe_full,
            run_move_group_node,
            move_named,
        ]
    )
