from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    ee_ip = LaunchConfiguration("ee_ip")
    arm = LaunchConfiguration("arm")
    mount = LaunchConfiguration("mount")
    sensor = LaunchConfiguration("sensor")
    controller = LaunchConfiguration("controller")
    tool_change_hardware = LaunchConfiguration("tool_change_hardware")
    end_effector = LaunchConfiguration("end_effector")
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur20",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ee_ip", description="IP address by which the end effector can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm", description="Robot arm (e.g., ur5e, tm5-700, etc., ...)"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mount", description="Mouting hardware (e.g., mpm)"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sensor", description="Sensor device (e.g., d435i)"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller", description="Peripheral controller (e.g., rpi5)"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_change_hardware", description="Tool change hardware (e.g., millibar)"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "end_effector", description="End Effector (e.g., robotiq_2f85)"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    return LaunchDescription(
        declared_arguments
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ur_robot_driver"),
                                "launch",
                                "ur_control.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "robot_ip": robot_ip,
                    "ee_ip": ee_ip,
                    "arm": arm,
                    "mount": mount,
                    "sensor": sensor,
                    "controller": controller,
                    "tool_change_hardware": tool_change_hardware,
                    "end_effector": end_effector,
                    "tf_prefix": [LaunchConfiguration("ur_type"), "_"],
                    "rviz_config_file": PathJoinSubstitution(
                        [
                            FindPackageShare("armada_description"),
                            "rviz",
                            "urdf.rviz",
                        ]
                    ),
                    "description_launchfile": PathJoinSubstitution(
                        [
                            FindPackageShare("armada_bringup"),
                            "launch",
                            "modules",
                            "arms",
                            "ur5e",
                            "rsp.launch.py",
                        ]
                    ),
                }.items(),
            ),
        ]
    )