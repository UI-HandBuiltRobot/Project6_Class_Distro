from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    search_aruco_id_arg = DeclareLaunchArgument(
        'search_aruco_id',
        default_value='0',
        description='ArUco marker ID used for box search/approach.',
    )
    search_aruco_id = LaunchConfiguration('search_aruco_id')

    sys_manager = Node(
        package='system_manager_package',
        executable='state_manager.py',
        name='state_manager_node',
        output='both',
        parameters=[{'search_aruco_id': search_aruco_id}],
    )

    return LaunchDescription([
        search_aruco_id_arg,
        sys_manager,
    ])

