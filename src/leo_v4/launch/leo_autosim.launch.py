from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()

    # Include SLAM Toolbox standard launch file
    launch_nav = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('leo_v2'), '/launch', '/leo_autonav.launch.py']),
    launch_arguments={}.items(),
    )

    launch_control = Node(
        package='leo_v4',
        executable='autonav_starter',
        name='autonav_starter',
        output='screen',
    )

    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(launch_nav)
    ld.add_action(launch_control)
    return ld