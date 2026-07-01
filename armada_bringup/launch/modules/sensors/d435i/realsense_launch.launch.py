from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    parent_frame = LaunchConfiguration("parent_frame")
    child_frame = LaunchConfiguration("child_frame")

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "examples",
                "pointcloud",
                "rs_pointcloud_launch.py",
            )
        ),
        launch_arguments={
            "enable_color": "true",
            "enable_depth": "true",
            "pointcloud.enable": "true",
            "align_depth.enable": "false",
            "enable_sync": "true",
            "enable_rgbd": "false",
        }.items(),
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="d435i_static_tf_publisher",
        arguments=[
            x, y, z,
            roll, pitch, yaw,
            parent_frame,
            child_frame,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("parent_frame", default_value="d435i_link"),
        DeclareLaunchArgument("child_frame", default_value="camera_link"),

        DeclareLaunchArgument("x", default_value="0"),
        DeclareLaunchArgument("y", default_value="0"),
        DeclareLaunchArgument("z", default_value="0"),
        DeclareLaunchArgument("roll", default_value="0"),
        DeclareLaunchArgument("pitch", default_value="0"),
        DeclareLaunchArgument("yaw", default_value="0"),

        realsense_launch,
        static_tf,
    ])