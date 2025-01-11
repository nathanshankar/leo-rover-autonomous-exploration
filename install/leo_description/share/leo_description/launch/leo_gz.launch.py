import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    ld = LaunchDescription()

    sdf_path = os.path.join(get_package_share_directory('leo_description'), 'worlds', 'tb3.sdf')

    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        gz_world_path =  os.environ['IGN_GAZEBO_RESOURCE_PATH'] + os.pathsep + os.path.join(get_package_share_directory('leo_description'), "worlds")
    else:
        gz_world_path =  os.path.join(get_package_share_directory('leo_description'), "worlds")

    ign_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',value=[gz_world_path])
    


    # Launch world
    gz_start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
        launch_arguments={
            'gz_args' : '-r ' + 'empty.sdf'
            }.items(),
    )

    # Add features
    gz_spawn_objects = Node(package='ros_gz_sim', executable='create',
    arguments=['-file', sdf_path,
    '-x', '2.0',
    '-y', '0.5',
    '-z', '0.0'],
    output='screen'
    )

    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('leo_description'), 'rviz', 'leo_view.rviz')]
    )

    # Add actions
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(ign_resource_path)
    ld.add_action(gz_spawn_objects)
    ld.add_action(gz_start_world)
    ld.add_action(node_rviz)

    return ld