#!/usr/bin/env python3
"""
Main bringup: sim (ros2_control) + MoveIt + synced camera + synced viewer + RViz.
"""
import os
import sys
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def _detect_python_and_env():
    """
    Choose python executable for MuJoCo nodes.

    Priority:
      1) If CONDA_PREFIX set and bin/python3 exists -> use conda python (+ LD_LIBRARY_PATH tweak)
      2) Else if VIRTUAL_ENV set and bin/python3 exists -> use venv python
      3) Else fall back to sys.executable (python running this launch file)
    """
    conda_prefix = os.environ.get("CONDA_PREFIX")
    if conda_prefix:
        conda_py = os.path.join(conda_prefix, "bin", "python3")
        if os.path.exists(conda_py):
            env = {
                # Ensure conda native libs (MuJoCo, etc.) are found first
                "LD_LIBRARY_PATH": os.path.join(conda_prefix, "lib")
                + ":" + os.environ.get("LD_LIBRARY_PATH", "")
            }
            return conda_py, env, "conda"

    venv_prefix = os.environ.get("VIRTUAL_ENV")
    if venv_prefix:
        venv_py = os.path.join(venv_prefix, "bin", "python3")
        if os.path.exists(venv_py):
            # Usually no LD_LIBRARY_PATH tweaks needed for venv
            return venv_py, {}, "venv"

    return sys.executable, {}, "sys"


def generate_launch_description():
    panda_sim_dir = get_package_share_directory('panda_sim')
    armada_bringup_dir = get_package_share_directory('armada_bringup')
    moveit_config_dir = get_package_share_directory('panda_moveit_config')

    # Detect default python (conda / venv / sys)
    python_exec_default, mujoco_env_default, python_kind = _detect_python_and_env()

    # Allow explicit override from the command line:
    #   ros2 launch ... python_exec:=/abs/path/to/python3
    python_exec_arg = DeclareLaunchArgument(
        "python_exec",
        default_value=python_exec_default,
        description="Python executable used to run MuJoCo Python nodes (auto-detect conda/venv; override if needed).",
    )
    python_exec = LaunchConfiguration("python_exec")

    # Log what we auto-detected (even if overridden, this still shows the default detection)
    python_info = LogInfo(
        msg=[
            "MuJoCo python auto-detected as (",
            python_kind,
            "): ",
            python_exec_default,
            "  (override with python_exec:=/path/to/python3)",
        ]
    )

    # -- 1) Simulation (robot_state_publisher + ros2_control + controller spawners) --
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(armada_bringup_dir, 'launch', 'mujoco_simulation.launch.py')
        )
    )

    # -- 2) MoveIt move_group --
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_dir, "launch", "move_group.launch.py")
        )
    )

    # -- 3) Synced camera node (replaces old mujoco_camera_node) --
    camera_sync = ExecuteProcess(
        cmd=[
            python_exec,  # <-- uses override if provided, else auto-detected default
            "-m",
            "panda_sim.mujoco_camera_sync_node",
            "--ros-args",
            "-p",
            "camera_name:=rgb_camera",
            "-p",
            "width:=640",
            "-p",
            "height:=480",
            "-p",
            "fps:=15.0",
            "-p",
            "frame_id:=mujoco_camera",
        ],
        output="screen",
        additional_env=mujoco_env_default,  # conda gets LD_LIBRARY_PATH; venv/system empty dict
    )

    # -- 4) Synced viewer node --
    viewer = ExecuteProcess(
        cmd=[
            python_exec,  # <-- uses override if provided, else auto-detected default
            "-m",
            "panda_sim.mujoco_viewer_node",
        ],
        output="screen",
        additional_env=mujoco_env_default,
    )

    # -- 5) RViz (delayed to let MoveIt start) --
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_dir, "launch", "moveit_rviz.launch.py")
        )
    )

    return LaunchDescription(
        [
            python_exec_arg,
            python_info,
            sim,
            moveit,
            # Start camera + viewer after controllers are up (~3s)
            TimerAction(period=3.0, actions=[camera_sync, viewer]),
            # Start RViz after MoveIt (~5s)
            TimerAction(period=5.0, actions=[rviz]),
        ]
    )
