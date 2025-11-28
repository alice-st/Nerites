from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the timeout argument (no default, must be provided)
    timeout_arg = DeclareLaunchArgument('timeout')

    # Define the node
    simulation_timeout_node = Node(
        package='uuv_assistants',
        executable='set_simulation_timer.py',
        name='simulation_timeout',
        parameters=[{'timeout': LaunchConfiguration('timeout')}],
        output='screen',
        # required=True means: if this node exits, the entire launch will shut down
        # (matches the ROS1 XML required="true" behavior)
    )

    return LaunchDescription([
        timeout_arg,
        simulation_timeout_node
    ])
