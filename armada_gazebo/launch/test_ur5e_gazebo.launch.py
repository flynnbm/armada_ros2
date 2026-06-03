import os
import subprocess
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_robot_sdf(context, *args, **kwargs):
    armada_description_share = get_package_share_directory('armada_description')
    armada_gazebo_share = get_package_share_directory('armada_gazebo')

    robot_xacro = os.path.join(
        armada_description_share,
        'urdf',
        'robot.urdf.xacro'
    )

    world_xacro = os.path.join(
        armada_gazebo_share,
        'sdf',
        'worlds',
        'simple_world.sdf.xacro'
    )

    generated_urdf = os.path.join(tempfile.gettempdir(), 'armada_robot.urdf')
    generated_sdf = os.path.join(tempfile.gettempdir(), 'armada_robot.sdf')
    generated_world = os.path.join(tempfile.gettempdir(), 'armada_world.sdf')

    arm = LaunchConfiguration('arm').perform(context)
    mount = LaunchConfiguration('mount').perform(context)
    d435i = LaunchConfiguration('sensor').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    tool_change_hardware = LaunchConfiguration('tool_change_hardware').perform(context)
    end_effector = LaunchConfiguration('end_effector').perform(context)

    xacro_cmd = [
        'ros2', 'run', 'xacro', 'xacro',
        robot_xacro,
        f'arm:={arm}',
        f'mount:={mount}',
        f'sensor:={d435i}',
        f'controller:={controller}',
        f'tool_change_hardware:={tool_change_hardware}',
        f'end_effector:={end_effector}',
        f'root_link:=world',
    ]

    with open(generated_urdf, 'w') as urdf_file:
        subprocess.run(xacro_cmd, stdout=urdf_file, check=True)

    with open(generated_sdf, 'w') as sdf_file:
        subprocess.run(
            ['gz', 'sdf', '-p', generated_urdf],
            stdout=sdf_file,
            check=True
        )
    
    with open(generated_world, 'w') as world_file:
        subprocess.run(
            ['ros2', 'run', 'xacro', 'xacro', world_xacro],
            stdout=world_file,
            check=True
        )

    with open(generated_sdf, 'r') as robot_file:
        robot_xml = robot_file.read()

    with open(generated_world, 'r') as world_file:
        world_xml = world_file.read()

    model_start = robot_xml.find('<model ')
    model_end = robot_xml.rfind('</model>') + len('</model>')

    if model_start == -1 or model_end == -1:
        raise RuntimeError('Could not find <model>...</model> in generated robot SDF')

    robot_model_xml = robot_xml[model_start:model_end]

    world_xml = world_xml.replace(
        '</world>',
        robot_model_xml + '\n</world>'
    )

    with open(generated_world, 'w') as world_file:
        world_file.write(world_xml)

    return [
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', generated_world],
            output='screen'
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm", default_value="ur5e"),
        DeclareLaunchArgument("mount", default_value="mpm"),
        DeclareLaunchArgument("sensor", default_value="d435i"),
        DeclareLaunchArgument("controller", default_value="rpi5"),
        DeclareLaunchArgument("tool_change_hardware", default_value="millibar"),
        DeclareLaunchArgument("end_effector", default_value="robotiq_2f85"),

        OpaqueFunction(function=generate_robot_sdf),
    ])