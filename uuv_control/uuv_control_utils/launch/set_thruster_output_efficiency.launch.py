from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration

"""
ROS2 launch file for dynamically setting a thruster’s output efficiency
=======================================================================

This launch script starts a ROS2 node (`set_thruster_output_efficiency.py`)
from the `uuv_control_utils` package to adjust the efficiency scaling factor
of a specified thruster on a UUV. It supports delayed application and 
optional duration for the efficiency change.

Launch Arguments:
-----------------
- uuv_name:       Vehicle namespace to push all topics/services into.
- starting_time:  Time (in seconds) after simulation start to apply the efficiency change.
- thruster_id:    ID of the thruster to adjust (integer).
- efficiency:     Desired efficiency scaling factor (e.g., 0.8 = 80%).
- duration:       How long to maintain the efficiency change (-1 = indefinite).

Example usage:
--------------
    ros2 launch uuv_control_utils set_thruster_output_efficiency.launch.py uuv_name:=rexrov starting_time:=5 thruster_id:=2 efficiency:=0.7 duration:=15
"""



def launch_setup(context, *args, **kwargs):
    """Build the thruster efficiency node dynamically after resolving substitutions."""
    uuv_name = LaunchConfiguration('uuv_name').perform(context)
    starting_time = LaunchConfiguration('starting_time').perform(context)
    thruster_id = LaunchConfiguration('thruster_id').perform(context)
    efficiency = LaunchConfiguration('efficiency').perform(context)
    duration = LaunchConfiguration('duration').perform(context)

    # Build the dynamic node name
    dynamic_name = f"set_thruster_output_efficiency_{starting_time}_thruster_{thruster_id}"

    # Define the Node with resolved parameters
    thruster_eff_node = Node(
        package='uuv_control_utils',
        executable='set_thruster_output_efficiency.py',
        name=dynamic_name,
        output='screen',
        parameters=[{
            'starting_time': float(starting_time),
            'thruster_id': int(thruster_id),
            'efficiency': float(efficiency),
            'duration': float(duration)
        }]
    )

    # Namespace group
    group = GroupAction([
        PushRosNamespace(uuv_name),
        thruster_eff_node
    ])

    return [group]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name', description='Vehicle namespace'),
        DeclareLaunchArgument('starting_time', default_value='0.0', description='When to start applying the efficiency change'),
        DeclareLaunchArgument('thruster_id', default_value='0', description='ID of the thruster'),
        DeclareLaunchArgument('efficiency', default_value='1.0', description='Output efficiency scaling factor'),
        DeclareLaunchArgument('duration', default_value='-1', description='Duration for the change (-1 means indefinite)'),

        OpaqueFunction(function=launch_setup)
    ])
