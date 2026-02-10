import os
import xacro
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, "r") as f:
        return f.read()


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, "r") as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    # --- Common args ---
    robot_make = LaunchConfiguration("robot_make").perform(context)
    robot_model = LaunchConfiguration("robot_model").perform(context)
    robot_source = LaunchConfiguration("robot_source").perform(context)
    workstation = LaunchConfiguration("workstation").perform(context)

    simulator = LaunchConfiguration("simulator").perform(context)

    # Mujoco
    mujoco_launch_path = os.path.join(
        get_package_share_directory("panda_sim"),
        "launch",
        "bringup.launch.py",
    )

    include_mujoco_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mujoco_launch_path),
        # no args needed right now
        condition=IfCondition(PythonExpression([f"'{simulator}' == 'mujoco'"])),
    )

    # Gazebo
    base_launch_path = os.path.join(
        get_package_share_directory("armada_bringup"),
        "launch",
        "gazebo_move_group.launch.py",
    )

    include_gazebo_move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path),
        launch_arguments={
            "robot_make": LaunchConfiguration("robot_make"),
            "robot_model": LaunchConfiguration("robot_model"),
            "robot_source": LaunchConfiguration("robot_source"),
            "workstation": LaunchConfiguration("workstation"),
        }.items(),
        condition=IfCondition(PythonExpression([f"'{simulator}' == 'gazebo'"])),
    )

    # FlexBE
    description_package = f"{robot_source}_description"
    moveit_config_package = f"{robot_model}_moveit_config"
    gazebo_package = f"{robot_source}_gazebo"

    robot_description_pkg = get_package_share_directory(description_package)
    moveit_config_path = get_package_share_directory(moveit_config_package)
    gazebo_package_path = get_package_share_directory(gazebo_package)
    flexbe_webui_path = get_package_share_directory("flexbe_webui")

    # Robot Description
    xacro_path = os.path.join(
        robot_description_pkg,
        f"{robot_model}",
        "xacro",
        f"{robot_model}" + (f"_{workstation}" if workstation else "") + ".urdf.xacro",
    )
    robot_description_config = xacro.process_file(xacro_path)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # SRDF
    robot_description_semantic_config = load_file(
        moveit_config_package, f"config/{robot_model}.srdf"
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    # OMPL planning
    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization \
                                    default_planner_request_adapters/FixWorkspaceBounds \
                                    default_planner_request_adapters/FixStartStateBounds \
                                    default_planner_request_adapters/FixStartStateCollision \
                                    default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        },
    }
    ompl_planning_yaml = load_yaml(moveit_config_package, "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    ompl_planning_pipeline_config_mtc = dict(ompl_planning_pipeline_config)
    ompl_planning_pipeline_config_mtc["ompl"] = dict(ompl_planning_pipeline_config["ompl"])
    req_adapters = ompl_planning_pipeline_config_mtc["ompl"].get("request_adapters", [])
    if isinstance(req_adapters, str):
        ompl_planning_pipeline_config_mtc["ompl"]["request_adapters"] = req_adapters.split()

    planning_group = f"{robot_make}_arm"

    # FlexBE full bringup OCS + onboard
    flexbe_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(flexbe_webui_path, "launch", "flexbe_full.launch.py")
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_flexbe")),
    )

    # Optional sim camera + TF
    spawn_camera = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file", f"{gazebo_package_path}/rgbd_camera/model/rgbd_camera_model.sdf",
            "-name", "rgbd_camera",
            "-x", "0.0", "-y", "0.0", "-z", "1.0",
            "-R", "0.0", "-P", "0.7854", "-Y", "0.0",
        ],
        output="screen",
        condition=IfCondition(
            PythonExpression([
                f"'{simulator}' == 'gazebo' and '",
                LaunchConfiguration("spawn_sim_camera"),
                "' == 'True'"
            ])
        )
    )

    sim_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.0", "0", "0.75", "0", "0.7854", "0",
            "simple_pedestal", "rgbd_camera/camera_link/rgbd_camera",
        ],
        condition=IfCondition(
            PythonExpression([
                f"'{simulator}' == 'gazebo' and '",
                LaunchConfiguration("spawn_sim_camera"),
                "' == 'True'"
            ])
        )
    )

    # Optional: spawn some objects
    spawn_object0 = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file", f"{robot_description_pkg}/simple_objects/wood_cylinder_flared_1_25cm/model.sdf",
            "-name", "object_0",
            "-x", "0.55", "-y", "0.075", "-z", "0.685",
            "-R", "0.0", "-P", "0.0", "-Y", "0.0",
        ],
        output="screen",
        condition=IfCondition(
            PythonExpression([
                f"'{simulator}' == 'gazebo' and '",
                LaunchConfiguration("spawn_objects"),
                "' == 'True'"
            ])
        )
    )

    spawn_object1 = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file", f"{robot_description_pkg}/simple_objects/wood_cylinder_flared_1_25cm/model.sdf",
            "-name", "object_1",
            "-x", "0.75", "-y", "-0.18", "-z", "0.685",
            "-R", "0.0", "-P", "0.0", "-Y", "0.0",
        ],
        output="screen",
        condition=IfCondition(
            PythonExpression([
                f"'{simulator}' == 'gazebo' and '",
                LaunchConfiguration("spawn_objects"),
                "' == 'True'"
            ])
        )
    )

    spawn_object2 = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file", f"{robot_description_pkg}/simple_objects/wood_cylinder_flared_1_25cm/model.sdf",
            "-name", "object_2",
            "-x", "0.65", "-y", "0.225", "-z", "0.685",
            "-R", "0.0", "-P", "0.0", "-Y", "0.0",
        ],
        output="screen",
        condition=IfCondition(
            PythonExpression([
                f"'{simulator}' == 'gazebo' and '",
                LaunchConfiguration("spawn_objects"),
                "' == 'True'"
            ])
        )
    )

    return [
        include_gazebo_move_group,
        include_mujoco_bringup,
        flexbe_full,
        spawn_camera,
        sim_camera_tf,
        spawn_object0,
        spawn_object1,
        spawn_object2,
    ]


def generate_launch_description():
    return LaunchDescription([
        # --- existing args forwarded to the base launch ---
        DeclareLaunchArgument("robot_make", default_value="panda"),
        DeclareLaunchArgument("robot_model", default_value="panda"),
        DeclareLaunchArgument("robot_source", default_value="armada"),
        DeclareLaunchArgument("workstation", default_value="simple_pedestal"),

        DeclareLaunchArgument(
            "simulator",
            default_value="gazebo",
            description="Which simulator include to use.",
        ),

        # --- flexbe args ---
        DeclareLaunchArgument(
            "launch_flexbe",
            default_value="False",
            description="If true, start FlexBE (onboard + OCS).",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="False",
            description="Run FlexBE OCS without the web UI frontend.",
        ),

        # --- camera args ---
        DeclareLaunchArgument(
            "spawn_sim_camera",
            default_value="True",
            description="Spawn the RGBD camera model and static TF in simulation.",
        ),
        DeclareLaunchArgument(
            "spawn_objects",
            default_value="True",
            description="Spawn sample objects into the scene.",
        ),

        OpaqueFunction(function=launch_setup),
    ])