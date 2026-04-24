
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # --- XArm access mode (uncomment ONE pair) ---
    # Option A: service-based (recommended) — xarm_hardware_node owns the USB
    #           and other nodes request motion via ROS2 services/actions.
    xarm_hardware = Node(
        package='xarm_object_collector_package',
        executable='xarm_hardware_node.py',
        name='xarm_hardware_node',
        output='both',
    )
    # Option B: direct class import — the action server imports XARMController
    #           itself.  Do NOT run both; they will fight over the USB device.
    # xarm_hardware = Node(
    #     package='xarm_object_collector_package',
    #     executable='xarm_hardware_node.py',
    #     name='xarm_hardware_node',
    #     output='both',
    # )

    xarm_action_commander = Node(
        package='xarm_object_collector_package',
        executable='Object_collector_action_server.py',
        name='grasping_commander_action_node',
        output='both',
    )

    q_learning = Node(
        package='xarm_object_collector_package',
        executable='q_learning_hand.py',
        name='q_learning_wrist_node',
        output='both',
    )

    return LaunchDescription([
        xarm_hardware,  # Comment out this line for Option B (direct class import)
        xarm_action_commander,
        q_learning,
    ])


