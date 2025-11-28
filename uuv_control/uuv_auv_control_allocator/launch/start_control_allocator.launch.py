from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # -------------------------
    # Declare launch arguments
    # -------------------------
    args = [
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('base_link', default_value='base_link'),
        DeclareLaunchArgument('output_dir'),
        DeclareLaunchArgument('input_topic', default_value='control_allocation/control_input'),

        # Thruster configuration
        DeclareLaunchArgument('thruster_topic_prefix', default_value='thrusters'),
        DeclareLaunchArgument('thruster_topic_suffix', default_value='input'),
        DeclareLaunchArgument('thruster_frame_base', default_value='thruster_'),
        DeclareLaunchArgument('max_thrust', default_value='120'),
        DeclareLaunchArgument('thruster_conversion_fcn', default_value='proportional'),
        DeclareLaunchArgument('thruster_gain', default_value='0.0'),
        DeclareLaunchArgument('thruster_input', default_value='0,1,2,3'),
        DeclareLaunchArgument('thruster_output', default_value='0,1,2,3'),

        # Fin configuration
        DeclareLaunchArgument('fin_frame_base', default_value='fin'),
        DeclareLaunchArgument('fluid_density', default_value='1028.0'),
        DeclareLaunchArgument('lift_coefficient', default_value='0.0'),
        DeclareLaunchArgument('fin_area', default_value='0.0'),
        DeclareLaunchArgument('fin_topic_prefix', default_value='fins'),
        DeclareLaunchArgument('fin_topic_suffix', default_value='input'),
        DeclareLaunchArgument('fin_lower_joint_limit', default_value='-1.57'),
        DeclareLaunchArgument('fin_upper_joint_limit', default_value='1.57'),

        # General
        DeclareLaunchArgument('timeout', default_value='-1'),
        DeclareLaunchArgument('update_rate', default_value='10'),
    ]

    # -------------------------
    # Define the node
    # -------------------------
    control_allocator_node = Node(
        package='uuv_auv_control_allocator',
        executable='control_allocator',
        name='control_allocator',
        output='screen',
        # Remap topic
        remappings=[('control_input', LaunchConfiguration('input_topic'))],
        # Hierarchical parameters
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir'),
            'base_link': LaunchConfiguration('base_link'),
            'thruster_config': {
                'topic_prefix': LaunchConfiguration('thruster_topic_prefix'),
                'topic_suffix': LaunchConfiguration('thruster_topic_suffix'),
                'frame_base': LaunchConfiguration('thruster_frame_base'),
                'max_thrust': LaunchConfiguration('max_thrust'),
                'conversion_fcn': LaunchConfiguration('thruster_conversion_fcn'),
                'conversion_fcn_params': {
                    'gain': LaunchConfiguration('thruster_gain'),
                    'input': [LaunchConfiguration('thruster_input')],
                    'output': [LaunchConfiguration('thruster_output')],
                }
            },
            'fin_config': {
                'frame_base': LaunchConfiguration('fin_frame_base'),
                'fluid_density': LaunchConfiguration('fluid_density'),
                'lift_coefficient': LaunchConfiguration('lift_coefficient'),
                'fin_area': LaunchConfiguration('fin_area'),
                'topic_prefix': LaunchConfiguration('fin_topic_prefix'),
                'topic_suffix': LaunchConfiguration('fin_topic_suffix'),
                'lower_limit': LaunchConfiguration('fin_lower_joint_limit'),
                'upper_limit': LaunchConfiguration('fin_upper_joint_limit'),
            },
            'timeout': LaunchConfiguration('timeout'),
            'update_rate': LaunchConfiguration('update_rate'),
        }]
    )

    # -------------------------
    # Namespace grouping
    # -------------------------
    group_action = GroupAction([
        PushRosNamespace(LaunchConfiguration('uuv_name')),
        control_allocator_node
    ])

    # -------------------------
    # Return full launch description
    # -------------------------
    return LaunchDescription(args + [group_action])
