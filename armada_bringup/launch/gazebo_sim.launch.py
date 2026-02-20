import os
import sys
import xacro
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter, LoadComposableNodes, ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except OSError:
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except OSError:
        return None

def launch_setup(context, *args, **kwargs):
    robot_make = LaunchConfiguration('robot_make').perform(context)
    robot_model = LaunchConfiguration('robot_model').perform(context)
    robot_source = LaunchConfiguration('robot_source').perform(context)
    gazebo_package = f"{robot_source}_gazebo"

    gazebo_package_path = get_package_share_directory(gazebo_package)
    ros_gz_sim_path = get_package_share_directory('ros_gz_sim')

    description_package = "robotiq_description"
    robot_description_pkg = get_package_share_directory(description_package)

    # Robot Description
    xacro_path = os.path.join(robot_description_pkg,  "urdf", "robotiq_2f_85_gripper.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # Controllers
    controllers_yaml = load_yaml(description_package, 'config/robotiq_controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }

    # Other params
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.1,
    }

    # Launch gz_sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_path, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[
            ('gz_args', f"-r {gazebo_package_path}/{robot_model}/worlds/{robot_model}.sdf --physics-engine gz-physics-bullet-featherstone-plugin")
        ],
    )

    # Spawn the robot model into the gazebo world
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", robot_model,
            "-topic", "/robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.25",
        ],
        output="screen",
    )

    gz_topic_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(gazebo_package_path, f'{robot_model}', 'config', f'{robot_model}_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    gz_services_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_services',
        # Use CLI-style mappings as arguments (most robust across distros)
        arguments=[
            '/world/panda/create@ros_gz_interfaces/srv/SpawnEntity',
            '/world/panda/remove@ros_gz_interfaces/srv/DeleteEntity',
            '/world/panda/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
        output='screen'
    )

    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # run_move_group_node = Node(
    #     package='moveit_ros_move_group',
    #     executable='move_group',
    #     output='screen',
    #     emulate_tty=True,
    #     parameters=[
    #         robot_description,
    #         # robot_description_semantic,
    #         # robot_description_kinematics,
    #         # ompl_planning_pipeline_config,
    #         trajectory_execution,
    #         moveit_controllers,
    #         # planning_scene_monitor_parameters,
    #         # joint_limits_yaml,
    #         {"use_sim_time": True},
    #         move_group_capabilities,
    #     ],
    # )

    load_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            'robotiq_gripper_controller',
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    # start up all of the nodes
    return [
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        gz_topic_bridge,
        gz_services_bridge,
        load_gripper_controller,
        # run_move_group_node,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_make',
            default_value='panda',
            description='Name of the robot make (panda, ur, tmr, etc...) to launch (used to locate packages and files).'
        ),
        DeclareLaunchArgument(
            'robot_model',
            default_value='panda',
            description='Name of the robot model (panda, ur5e, tm5-700, etc...) to launch (used to locate packages and files).'
        ),
        DeclareLaunchArgument(
            'robot_source',
            default_value='armada',
            description='Name of the organization the robot belongs to (Universal Robot, Armada, concept in development...) (used to locate packages and files).'
        ),
        DeclareLaunchArgument(
            'workstation',
            default_value='simple_pedestal',
            description='Name of the pedestal or workstation the robot base is attached to (used to locate packages and files).'
        ),
        OpaqueFunction(function=launch_setup),
    ])