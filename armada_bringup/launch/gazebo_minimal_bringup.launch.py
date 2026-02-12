from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch.actions import ExecuteProcess

ur_share = PathJoinSubstitution([FindPackageShare("ur_description"), ".."])
robotiq_share = PathJoinSubstitution([FindPackageShare("robotiq_description"), ".."])

def generate_launch_description():
    robot_model = LaunchConfiguration("robot_model")
    end_effector = LaunchConfiguration("end_effector")
    z = LaunchConfiguration("spawn_z")

    # Packages
    description_pkg = FindPackageShare("armada_description")
    gazebo_pkg = FindPackageShare("armada_gazebo")
    ros_gz_sim_pkg = FindPackageShare("ros_gz_sim")

    # Xacro root
    description_file = PathJoinSubstitution([description_pkg, "urdf", "robot.urdf.xacro"])
    # description_file = PathJoinSubstitution([description_pkg, "old", "panda", "xacro", "panda.urdf.xacro"])

    # Robot description (IMPORTANT: enable ros2_control for sim)
    robot_description = ParameterValue(
        Command([
            "xacro ", description_file,
            " model:=", robot_model,
            " end_effector:=", end_effector,
            " use_ros2_control:=true",
            " sim_gazebo:=true",
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
    )

    # World: reuse your existing convention (armada_gazebo/<robot_model>/worlds/<robot_model>.sdf)
    world_sdf = PathJoinSubstitution([gazebo_pkg, robot_model, "worlds", PathJoinSubstitution([robot_model, ".sdf"])])

    # NOTE: PathJoinSubstitution can't append ".sdf" that way nicely in all cases.
    # If you prefer, hardcode a world arg or just use a default world.
    # To keep this robust, we’ll take a 'world' launch arg instead:
    world_arg = LaunchConfiguration("world")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_pkg, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": ["-r ", world_arg, " --physics-engine gz-physics-bullet-featherstone-plugin"]
        }.items(),
    )

    # Spawn robot into Gazebo from robot_description topic
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", robot_model,
            "-topic", "/robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", z,
        ],
    )

    # Controllers (spawn after Gazebo + entity are up)
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    spawn_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )

    spawn_controllers_delayed = TimerAction(
        period=3.0,  # avoids racing controller_manager startup inside gz_ros2_control
        actions=[
            spawn_joint_state_broadcaster,
            spawn_arm_controller,
            spawn_gripper_controller,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_model", default_value="ur5e"),
        DeclareLaunchArgument("end_effector", default_value="robotiq_2f85"),
        DeclareLaunchArgument("spawn_z", default_value="0.0"),

        # Strongly recommended: explicitly pass a world file path (string)
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([gazebo_pkg, "panda", "worlds", "panda.sdf"]),
            description="Absolute path to a Gazebo world SDF file.",
        ),

        gz_sim,
        robot_state_publisher,
        spawn_entity,
        spawn_controllers_delayed,
    ])