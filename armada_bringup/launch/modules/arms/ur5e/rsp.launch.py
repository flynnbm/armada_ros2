from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description():
    arm_ip = LaunchConfiguration("arm_ip")
    ee_ip = LaunchConfiguration("ee_ip")
    arm = LaunchConfiguration("arm")
    mount = LaunchConfiguration("mount")
    sensor = LaunchConfiguration("sensor")
    controller = LaunchConfiguration("controller")
    tool_change_hardware = LaunchConfiguration("tool_change_hardware")
    end_effector = LaunchConfiguration("end_effector")


    # Load description with necessary parameters
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("armada_description"),
                    "urdf",
                    "robot.urdf.xacro",
                ]
            ),
            " ",
            "arm_ip:=",
            arm_ip,
            " ",
            "ee_ip:=",
            ee_ip,
            " ",
            "arm:=",
            arm,
            " ",
            "mount:=",
            mount,
            " ",
            "sensor:=",
            sensor,
            " ",
            "controller:=",
            controller,
            " ",
            "tool_change_hardware:=",
            tool_change_hardware,
            " ",
            "end_effector:=",
            end_effector,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_ip", description="IP address by which the robot arm can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ee_ip", description="IP address by which the end effector can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm", description=""
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mount", description=""
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sensor", description=""
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller", description=""
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_change_hardware", description=""
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "end_effector", description=""
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[robot_description],
            ),
        ]
    )