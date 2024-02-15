import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ld = LaunchDescription()


    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'leo_v1'
    file_subpath = 'urdf/leo.urdf.xacro'

    # Set ignition resource path (so it can find your world files)
    ign_resource_path_update = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',
        value=[os.path.join(get_package_share_directory(pkg_name), 'worlds')])

    # Add the models directory to Gazebo's model path
    models_directory = os.path.join(get_package_share_directory('leo_v3'), 'models')
    gazebo_model_path_update = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH',
        value=[models_directory])

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory('leo_v3'), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('leo_description'), '/launch', '/leo_gz.launch.py']),
        launch_arguments={}.items(),
    )
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
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

    teleop_twist_keyboard_node = Node(
            package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard_node', 
            remappings={('/cmd_vel', '/teleop_cmd_vel')},
            output='screen',
            prefix = 'xterm -e',
            )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            {
                'output_topic': '/leo/cmd_vel',
                'topics': {
                    'keyboard': {
                        'topic': '/teleop_cmd_vel',
                        'timeout': 0.5,
                        'priority': 15
                    },
                    'locks': {
                        'auto_nav': {
                            'topic': '/nav_cmd_vel',
                            'timeout': 0.0,
                            'priority': 10
                        }
                    }
                }
            }
        ],
        remappings={('/cmd_vel_out', '/leo/cmd_vel')},
    )
    # Bridge
    # https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=  [
                    '/clock'                           + '@rosgraph_msgs/msg/Clock'   + '[' + 'ignition.msgs.Clock',
                    '/model/leo/cmd_vel'  + '@geometry_msgs/msg/Twist'   + '@' + 'ignition.msgs.Twist',
                    '/model/leo/odometry' + '@nav_msgs/msg/Odometry'     + '[' + 'ignition.msgs.Odometry',
                    '/model/leo/scan'     + '@sensor_msgs/msg/LaserScan' + '[' + 'ignition.msgs.LaserScan',
                    '/model/leo/tf'       + '@tf2_msgs/msg/TFMessage' + '[' + 'ignition.msgs.Pose_V',
                    '/model/leo/imu'      + '@sensor_msgs/msg/Imu'       + '[' + 'ignition.msgs.IMU',
                    '/world/empty/model/leo/joint_state' + '@sensor_msgs/msg/JointState' + '[' + 'ignition.msgs.Model',
                    'model/leo/image' + '@sensor_msgs/msg/Image' + '[' + 'ignition.msgs.Image',
                    '/world/empty/model/leo/link/px150/gripper_link/sensor/depth/depth_image' + '@sensor_msgs/msg/Image' + '[' + 'ignition.msgs.Image',
                    '/world/empty/model/leo/link/px150/gripper_link/sensor/color/image' + '@sensor_msgs/msg/Image' + '[' + 'ignition.msgs.Image',
                    '/world/empty/model/leo/link/px150/gripper_link/sensor/depth/depth_image/points' + '@sensor_msgs/msg/PointCloud2' + '[' + 'ignition.msgs.PointCloudPacked',
                    '/model/leo/battery/linear_battery/state' + '@sensor_msgs/msg/BatteryState' + '[' + 'ignition.msgs.Battery',

                    ],
        parameters= [{'qos_overrides./leo_v1.subscriber.reliability': 'reliable'},{'qos_overrides./leo_v1.subscriber.durability': 'transient_local'}],
        remappings= [
                    ('/model/leo/cmd_vel',  '/leo/cmd_vel'),
                    ('/model/leo/odometry', '/odom'   ),
                    ('/model/leo/scan',     '/scan'   ),
                    ('/model/leo/tf',       '/tf'     ),
                    ('/model/leo/imu',      '/imu_raw'),
                    ('/world/empty/model/leo/joint_state', 'joint_states'),
                    ('/model/leo/image', '/image_raw'),
                    ('/world/empty/model/leo/link/px150/gripper_link/sensor/depth/depth_image', '/depth'),
                    ('/world/empty/model/leo/link/px150/gripper_link/sensor/depth/depth_image/points', '/depth/points'),
                    ('/world/empty/model/leo/link/px150/gripper_link/sensor/color/image', '/depth_image_raw'),
                    ('/model/leo/battery/linear_battery/state', '/battery_state')
                    ],
        output='screen'
    )

    # # joint state publisher node
    # node_joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    # )

    node_tf_depth_cam= Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'leo/px150/gripper_link/depth'],
    )
    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    #ld.add_action(declare_joy_vel)
    ld.add_action(ign_resource_path_update)
    print("GAZEBO_MODEL_PATH:", models_directory)
    ld.add_action(node_tf_depth_cam)
    ld.add_action(gazebo_model_path_update)
    ld.add_action(launch_gazebo)
    ld.add_action(node_spawn_entity)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_ros_gz_bridge)
    ld.add_action(teleop_twist_keyboard_node)
    # ld.add_action(node_joint_state_publisher)
    ld.add_action(twist_mux_node)
    return ld
