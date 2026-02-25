#!/usr/bin/env python3

import os
import xacro
import yaml
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

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

def _generate_scene_at_launch(pkg_share, config_rel_path, seed=None):
    """
    Run scene generation synchronously during launch-description construction.
    Guarantees current_scene.xml exists before any node tries to load it.
    """
    import json

    config_path = os.path.join(pkg_share, config_rel_path)
    if not os.path.isfile(config_path):
        raise FileNotFoundError(
            f"Scene config not found: {config_path}\n"
            f"Make sure the config file is installed to the package share directory."
        )

    # Make sure panda_sim is importable (works with both --symlink-install
    # and regular install, as long as the package is on PYTHONPATH after
    # sourcing install/setup.bash)
    from panda_sim.scene_generation import generate_scene_from_config

    with open(config_path, "r") as f:
        cfg = json.load(f)

    scene_cfg = cfg["scene"]

    # Resolve template and output paths relative to the installed share dir
    scene_cfg["template_scene"] = os.path.join(
        pkg_share, scene_cfg["template_scene"])
    scene_cfg["output_scene"] = os.path.join(
        pkg_share, scene_cfg["output_scene"])

    if seed is None:
        seed = cfg.get("experiment", {}).get("random_seed", None)
    if not isinstance(seed, int):
        seed = None

    instances = generate_scene_from_config(scene_cfg, seed=seed)

    print(f"[simulation.launch] Generated scene: {scene_cfg['output_scene']}")
    print(f"[simulation.launch] Placed {len(instances)} objects:")
    for inst in instances:
        p = inst["pos"]
        print(f"  {inst['name']:12s}  {inst['type']:20s}  "
              f"({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})")

    return scene_cfg["output_scene"]

def generate_launch_description():
    pkg_share     = get_package_share_directory('panda_sim')
    urdf_file     = os.path.join(pkg_share, 'mujoco_models',   'panda.urdf')
    mujoco_params = os.path.join(pkg_share, 'config', 'mujoco_params.yaml')
    controllers   = os.path.join(pkg_share, 'config', 'controllers.yaml')
    spawner_launch= os.path.join(pkg_share, 'launch', 'controllers_spawner.launch.py')

    # ------------------------------------------------------------------ #
    #  Launch argument: scene_config (optional)                            #
    # ------------------------------------------------------------------ #
    scene_config_arg = DeclareLaunchArgument(
        'scene_config',
        default_value='',
        description='Relative path (within panda_sim share) to a scene config JSON. '
                    'Leave empty to skip scene generation.'
    )

    # Read the argument value at description-construction time, because
    # we need the scene XML to exist BEFORE nodes are constructed.
    scene_config_val = ''
    for arg in sys.argv:
        if arg.startswith('scene_config:='):
            scene_config_val = arg.split(':=', 1)[1]

    # ------------------------------------------------------------------ #
    #  Scene generation (synchronous)                                      #
    # ------------------------------------------------------------------ #
    info_actions = []
    if scene_config_val:
        model_xml = _generate_scene_at_launch(pkg_share, scene_config_val)
        info_actions.append(
            LogInfo(msg=f'Using generated scene: {model_xml}')
        )
    else:
        info_actions.append(
            LogInfo(msg='No scene_config provided; using default model from URDF.')
        )

    # load URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    use_sim_time = False

    # 1) Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description},
        ],
    )

    # 2) MuJoCo sim node via python module
    sim_process = ExecuteProcess(
        cmd=[
            'python3', '-m', 'panda_sim.mujoco_sim_node',
            '--ros-args', '--params-file', mujoco_params
        ],
        output='screen'
    )


    # 2) Controller manager
    cm_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description},
            controllers,
        ],
    )

    # ------------------------------------------------------------
    # adding in depth + rgb to pointcloud and static transform nodes
    # ------------------------------------------------------------

    # generate a pointcloud from the depth image
    point_cloud_xyz_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='point_cloud_xyz_node',
        # output='screen',
        remappings=[
            ('image_rect', '/camera/depth/image_raw'),
            ('camera_info', '/camera/camera_info'),
            ('points', '/camera/depth/points'),
        ],
    )

    # pointcloud node expects /camera/camera_info to also be in /camera/depth/ namespace
    camera_info_relay = Node(
        package='topic_tools',
        executable='relay',
        name='camera_info_relay',
        # output='screen',
        arguments=['/camera/camera_info', '/camera/depth/camera_info'],
    )

    # static transform from panda_hand to d435i
    tf_panda_hand_to_d435i = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_panda_hand_to_d435i',
        # output='screen',
        arguments=[
            '0.045', '0', '0.0075',          # x y z
            '0', '0', '0.707107', '0.707107',# qx qy qz qw
            'panda_hand', 'd435i',           # parent child
        ],
    )

    # static transform from d435i to mujoco camera
    tf_d435i_to_mujoco_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_d435i_to_mujoco_camera',
        # output='screen',
        arguments=[
            '0', '0', '0',                   # x y z
            '0', '0', '1', '0',              # qx qy qz qw
            'd435i', 'mujoco_camera',        # parent child
        ],
    )

    # ------------------------------------------------------------
    # end of camera pointcloud and transform nodes
    # ------------------------------------------------------------

    # ------------------------------------------------------------
    # adding in FlexBE Full launch here
    # ------------------------------------------------------------

    # FlexBE WebUI
    flexbe_webui_path = get_package_share_directory('flexbe_webui')

    # Launch FlexBE operator control system (OCS)
    flexbe_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(flexbe_webui_path, 'launch', 'flexbe_full.launch.py')),
        launch_arguments={
            'headless': 'True'
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_flexbe')),
    )

    # ------------------------------------------------------------
    # end of FlexBE
    # ------------------------------------------------------------

    # ------------------------------------------------------------
    # adding in all of my service server and action server nodes here
    # ------------------------------------------------------------

    home_dir = os.path.expanduser("~")
    gpd_library_path = os.path.join(home_dir, 'flexbe_ws', 'gpd') 

    # Planning Group
    planning_group = "arm"

    # MoveIt config
    moveit_config_package = "panda_moveit_config"

    # SRDF
    robot_description_semantic_config = load_file(moveit_config_package, f'config/panda.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    # Kinematics
    kinematics_yaml = load_yaml(moveit_config_package, 'config/kinematics.yaml')

    # Joint Limits
    joint_limits_yaml = {
        'robot_description_planning': load_yaml(
            moveit_config_package, 'config/joint_limits.yaml')
    }

    # OMPL Planning Parameters
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
    ompl_planning_pipeline_config_mtc = dict(ompl_planning_pipeline_config)
    ompl_planning_pipeline_config_mtc['ompl'] = dict(ompl_planning_pipeline_config['ompl'])

    # If request_adapters is a string, split it into a list
    req_adapters = ompl_planning_pipeline_config_mtc['ompl'].get('request_adapters', [])
    if isinstance(req_adapters, str):
        # split on whitespace into a list of class names
        ompl_planning_pipeline_config_mtc['ompl']['request_adapters'] = req_adapters.split()

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

    add_collision_object_service = Node(
        package="planning_scene_ros2",
        executable="add_collision_object_service",
        name="add_collision_object_service",
        output="screen",
        parameters=[
            {"frame_id": "world"},
        ],
    )

    # ------------------------------------------------------------
    # end of my nodes
    # ------------------------------------------------------------

    # 4) Include controller spawner launch
    spawner_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawner_launch)
    )

    return LaunchDescription([
        rsp_node,
        sim_process,
        cm_node,
        point_cloud_xyz_node,
        camera_info_relay,
        tf_panda_hand_to_d435i,
        tf_d435i_to_mujoco_camera,
        spawner_include,
        flexbe_full,
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
        add_collision_object_service,
    ])
