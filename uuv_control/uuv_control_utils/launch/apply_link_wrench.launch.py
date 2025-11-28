from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def launch_setup(context, *args, **kwargs):
    # --- Resolve launch arguments ---
    uuv_name = LaunchConfiguration('uuv_name').perform(context)
    force_x = LaunchConfiguration('force_x').perform(context)
    force_y = LaunchConfiguration('force_y').perform(context)
    force_z = LaunchConfiguration('force_z').perform(context)
    torque_x = LaunchConfiguration('torque_x').perform(context)
    torque_y = LaunchConfiguration('torque_y').perform(context)
    torque_z = LaunchConfiguration('torque_z').perform(context)
    starting_time = LaunchConfiguration('starting_time').perform(context)
    duration = LaunchConfiguration('duration').perform(context)
    gazebo_ns = LaunchConfiguration('gazebo_ns').perform(context)

    # --- Build safe static strings ---
    node_name = f"apply_link_wrench_{starting_time}_{force_x}_{force_y}_{force_z}_{torque_x}_{torque_y}_{torque_z}"
    force_param = f"{force_x},{force_y},{force_z}"
    torque_param = f"{torque_x},{torque_y},{torque_z}"

    # --- Define the node with evaluated params ---
    apply_wrench_node = Node(
        package='uuv_control_utils',
        executable='apply_link_wrench.py',
        name=node_name,
        output='screen',
        parameters=[{
            'starting_time': float(starting_time),
            'force': force_param,
            'torque': torque_param,
            'duration': float(duration),
            'gazebo_ns': gazebo_ns
        }]
    )

    # --- Namespace grouping ---
    return [GroupAction([
        PushRosNamespace(uuv_name),
        apply_wrench_node
    ])]

def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('force_x', default_value='0'),
        DeclareLaunchArgument('force_y', default_value='0'),
        DeclareLaunchArgument('force_z', default_value='0'),
        DeclareLaunchArgument('torque_x', default_value='0'),
        DeclareLaunchArgument('torque_y', default_value='0'),
        DeclareLaunchArgument('torque_z', default_value='0'),
        DeclareLaunchArgument('starting_time', default_value='0'),
        DeclareLaunchArgument('duration', default_value='1'),
        DeclareLaunchArgument('gazebo_ns', default_value='gazebo'),

        # Main logic
        OpaqueFunction(function=launch_setup)
    ])
