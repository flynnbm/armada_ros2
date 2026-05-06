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
    end_effector = LaunchConfiguration('end_effector').perform(context)
    workstation = LaunchConfiguration('workstation').perform(context)
    robot_source = LaunchConfiguration('robot_source').perform(context)

    description_package = f"{robot_source}_description"
    moveit_config_package = f"{robot_model}_{end_effector}_moveit_config"

    robot_description_pkg = get_package_share_directory(description_package)

    home_dir = os.path.expanduser("~")
    gpd_library_path = os.path.join(home_dir, 'flexbe_ws', 'gpd')                   # modify this command if the workspace where gpd is installed is different than flexbe_ws

    # Robot Description
    xacro_path = os.path.join(robot_description_pkg, f'{robot_model}', 'xacro', f'{robot_model}' + (f'_{workstation}' if workstation else '') + '.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_path)
    robot_description = {'robot_description': robot_description_config.toxml()} # type: ignore

    # SRDF
    robot_description_semantic_config = load_file(moveit_config_package, f'config/{robot_model}.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    # Kinematics
    kinematics_yaml = load_yaml(moveit_config_package, 'config/kinematics.yaml')

    # Planning Group
    planning_group = f"{robot_make}_arm"

    # OMPL Planning
    ompl_planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization \
                                    default_planner_request_adapters/FixWorkspaceBounds \
                                    default_planner_request_adapters/FixStartStateBounds \
                                    default_planner_request_adapters/FixStartStateCollision \
                                    default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
        },
    }
    ompl_planning_yaml = load_yaml(moveit_config_package, 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    #  MTC node OMPL config with adapters as a list
    ompl_planning_pipeline_config_mtc = dict(ompl_planning_pipeline_config)     # shallow copy ok for our keys
    ompl_planning_pipeline_config_mtc['ompl'] = dict(ompl_planning_pipeline_config['ompl'])

    # If request_adapters is a string, split it into a list
    req_adapters = ompl_planning_pipeline_config_mtc['ompl'].get('request_adapters', [])
    if isinstance(req_adapters, str):
        # split on whitespace into a list of class names
        ompl_planning_pipeline_config_mtc['ompl']['request_adapters'] = req_adapters.split()

    # Joint Limits
    joint_limits_yaml = {
        'robot_description_planning': load_yaml(
            moveit_config_package, 'config/joint_limits.yaml')
    }

    move_cartesian = Node(
        package="move_group_ros2",
        executable="cartesian_move_to_pose_service",
        name="cartesian_move_to_pose_service",
        output="screen",
        parameters=[
            {"planning_group": planning_group},
            robot_description,
            robot_description_semantic,
        ],
    )

    move_pose = Node(
        package="move_group_ros2",
        executable="move_to_pose_service",
        name="move_to_pose_service",
        output="screen",
        parameters=[
            {"planning_group": planning_group},
            robot_description,
            robot_description_semantic,
        ],
    )

    move_named = Node(
        package="move_group_ros2",
        executable="move_to_named_pose_service",
        name="move_to_named_pose_service",
        output="screen",
        parameters=[
            {"planning_group": planning_group},
            robot_description,
            robot_description_semantic,
        ],
    )

    mtc_plan_and_execute_pick = Node(
        package="mtc_ros2",
        executable="mtc_plan_and_execute_pick_action",
        name="mtc_plan_and_execute_pick_action",
        output="screen",
        parameters=[
            {"planning_group": planning_group},
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            joint_limits_yaml,
            ompl_planning_pipeline_config_mtc,
        ],
    )

    mtc_approach_and_pick = Node(
        package="mtc_ros2",
        executable="mtc_approach_and_pick_action",
        name="mtc_approach_and_pick_action",
        output="screen",
        parameters=[
            {"planning_group": planning_group},
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            joint_limits_yaml,
            ompl_planning_pipeline_config_mtc,
        ],
    )

    mtc_retreat_and_place = Node(
        package="mtc_ros2",
        executable="mtc_retreat_and_place_action",
        name="mtc_retreat_and_place_action",
        output="screen",
        parameters=[
            {"planning_group": planning_group},
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            joint_limits_yaml,
            ompl_planning_pipeline_config_mtc,
        ],
    )

    compute_grasp_poses = Node(
        package="gpd_ros",
        executable="grasp_pose_server",
        name="grasp_pose_server",
        output="screen",
        parameters=[
            {"gripper_offset": 0.041},
            {"approach_dist": 0.10},
            {"retreat_dist": 0.0},
            {"grasp_rot_x": 0.0},
            {"grasp_rot_y": 0.0},
            {"grasp_rot_z": 0.0},
            {"grasp_rot_w": 1.0},
            {"target_frame": "panda_link0"},
            {"source_frame": "panda_link0"},
        ],
    )

    get_pointcloud_service = Node(
        package="pcl_ros2",
        executable="get_point_cloud_service",
        name="get_point_cloud_service",
        output="screen",
        parameters=[
            {"default_camera_topic": "/camera/depth/points"},
            {"target_frame": "panda_link0"},
            {"timeout_sec": 3.0},
        ],
    )

    euclidean_clustering_service = Node(
        package="pcl_ros2",
        executable="euclidean_clustering_service",
        name="euclidean_clustering_service",
        output="screen",
        parameters=[
            {"default_camera_topic": "/camera/depth/points"},
            {"target_frame": "panda_link0"},
            {"timeout_sec": 3.0},
        ],
    )

    filter_by_indices_service = Node(
        package="pcl_ros2",
        executable="filter_by_indices_service",
        name="filter_by_indices_service",
        output="screen",
        parameters=[
            {"default_camera_topic": "/camera/depth/points"},
            {"target_frame": "panda_link0"},
            {"timeout_sec": 3.0},
        ],
    )

    plane_segmentation_service = Node(
        package="pcl_ros2",
        executable="plane_segmentation_service",
        name="plane_segmentation_service",
        output="screen",
    )

    passthrough_filter_service = Node(
        package="pcl_ros2",
        executable="passthrough_filter_service",
        name="passthrough_filter_service",
        output="screen",
    )

    detect_grasps = Node(
        package="gpd_ros",
        executable="grasp_detection_server",
        name="grasp_detection_server",
        # output="screen",
        parameters=[
            {"camera_position": [0.0, 0.0, 0.0]},
            {"config_file": f'{gpd_library_path}/cfg/ros_eigen_params.cfg'},
            {"grasps_topic": 'clustered_grasps'},
            # {"rviz_topic": "grasp_plotter"},
            {"service_name": 'detect_grasps'},
        ],
    )

    # start up all of the nodes
    return [
        move_cartesian,
        move_named,
        move_pose,
        mtc_plan_and_execute_pick,
        mtc_approach_and_pick,
        mtc_retreat_and_place,
        get_pointcloud_service,
        plane_segmentation_service,
        euclidean_clustering_service,
        filter_by_indices_service,
        passthrough_filter_service,
        detect_grasps,
        compute_grasp_poses,
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
            'end_effector',
            default_value='panda_hand',
            description='Name of the end effector model (panda_hand, robotiq_2f85, etc...) to launch (used to locate packages and files).'
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
        DeclareLaunchArgument(
            'headless', 
            default_value="False",
            description="Run FlexBE OCS without the web UI frontend."
        ),
        DeclareLaunchArgument(
            'launch_flexbe',
            default_value='False',
            description='If true, start FlexBE (onboard + OCS).'
        ),
        OpaqueFunction(function=launch_setup),
    ])