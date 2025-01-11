import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
import xacro


def generate_launch_description():
    ld = LaunchDescription()
    file_subpath = 'urdf/px150.urdf.xacro'
    sdf_path = os.path.join(get_package_share_directory('leo_v3'), 'worlds', 'tb3.sdf')
    xacro_file = os.path.join(get_package_share_directory('leo_v3'), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        gz_world_path =  os.environ['IGN_GAZEBO_RESOURCE_PATH'] + os.pathsep + os.path.join(get_package_share_directory('leo_v3'), "worlds")
    else:
        gz_world_path =  os.path.join(get_package_share_directory('leo_v3'), "worlds")

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

    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=  [
                    '/clock'                           + '@rosgraph_msgs/msg/Clock'   + '[' + 'ignition.msgs.Clock',
                    '/joint_trajectory@trajectory_msgs/msg/JointTrajectory]ignition.msgs.JointTrajectory',
                    '/world/empty/model/px150/joint_state' + '@sensor_msgs/msg/JointState' + '[' + 'ignition.msgs.Model',
                    '/joint_trajectory_progress'+'@std_msgs/msg/Float32'+'['+'ignition.msgs.Float',
                    ],
        parameters= [{'qos_overrides./leo.subscriber.reliability': 'reliable'}],
        remappings= [('/world/empty/model/px150/joint_state', 'joint_states'),
                    ],
        output='screen'
    )

    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('leo_v3'), 'rviz', 'leo_view.rviz')]
    )
    node_spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', '/robot_description',
                                   '-z', '0.5'],
                        output='screen')

    # robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )
    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )
    node_manipulator_control_gui = Node(
        package='leo_v3',
        executable='stretch_ignition_control_action_server',
        name='stretch_ignition_control',
        output='screen',
    )

    # Add actions
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(ign_resource_path)
    #ld.add_action(gz_spawn_objects)
    ld.add_action(gz_start_world)
    ld.add_action(node_spawn_entity)
    ld.add_action(node_robot_state_publisher)
    #ld.add_action(node_joint_state_publisher)  
    ld.add_action(node_ros_gz_bridge)
    ld.add_action(node_manipulator_control_gui)
    ld.add_action(node_rviz)

    return ld