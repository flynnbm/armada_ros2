import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import xacro


def generate_world(context, *args, **kwargs):
    pkg_share = get_package_share_directory('armada_gazebo')

    world_xacro = os.path.join(
        pkg_share,
        'sdf',
        'worlds',
        'simple_world.sdf.xacro'
    )

    arm = LaunchConfiguration('arm').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    robot_pose = LaunchConfiguration('robot_pose').perform(context)

    generated_world = os.path.join(
        tempfile.gettempdir(),
        f'{robot_name}_simple_world.sdf'
    )

    doc = xacro.process_file(
        world_xacro,
        mappings={
            'arm': arm,
            'robot_name': robot_name,
            'robot_pose': robot_pose,
        }
    )

    with open(generated_world, 'w') as f:
        f.write(doc.toprettyxml(indent='  '))

    return [
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', generated_world],
            output='screen'
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arm',
            default_value='simple_box_arm'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='armada_robot'
        ),
        DeclareLaunchArgument(
            'robot_pose',
            default_value='0 0 0.1 0 0 0'
        ),
        OpaqueFunction(function=generate_world),
    ])