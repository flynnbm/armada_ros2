from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm = LaunchConfiguration("arm")
    mount = LaunchConfiguration("mount")
    sensor = LaunchConfiguration("sensor")
    controller = LaunchConfiguration("controller")
    tool_change_hardware = LaunchConfiguration("tool_change_hardware")
    end_effector = LaunchConfiguration("end_effector")

    description_package = FindPackageShare("armada_description")
    description_file = PathJoinSubstitution(
        [description_package, "urdf", "robot.urdf.xacro"]
    )

    rvizconfig_file = PathJoinSubstitution([description_package, "rviz", "urdf.rviz"])

    robot_description = ParameterValue(
        Command([
            "xacro ", description_file,
            " arm:=", arm,
            " mount:=", mount,
            " sensor:=", sensor,
            " controller:=", controller,
            " tool_change_hardware:=", tool_change_hardware,
            " end_effector:=", end_effector,
        ]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizconfig_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument("arm", default_value="ur5e"),
        DeclareLaunchArgument("mount", default_value="mpm"),
        DeclareLaunchArgument("sensor", default_value="d435i"),
        DeclareLaunchArgument("controller", default_value="rpi5"),
        DeclareLaunchArgument("tool_change_hardware", default_value=""),
        DeclareLaunchArgument("end_effector", default_value="robotiq_2f85"),

        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ])
