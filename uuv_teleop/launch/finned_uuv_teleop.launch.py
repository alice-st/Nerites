#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    # Read all LaunchConfiguration values
    uuv_name = LaunchConfiguration('uuv_name').perform(context)
    joy_id = LaunchConfiguration('joy_id').perform(context)
    use_param_file = LaunchConfiguration('use_param_file').perform(context).lower() == 'true'
    filename = LaunchConfiguration('filename').perform(context)

    axis_thruster = LaunchConfiguration('axis_thruster').perform(context)
    axis_roll = LaunchConfiguration('axis_roll').perform(context)
    axis_pitch = LaunchConfiguration('axis_pitch').perform(context)
    axis_yaw = LaunchConfiguration('axis_yaw').perform(context)

    thruster_rotor_gain = LaunchConfiguration('thruster_rotor_gain').perform(context)
    max_thrust = LaunchConfiguration('max_thrust').perform(context)
    thruster_topic = LaunchConfiguration('thruster_topic').perform(context)
    fin_topic_prefix = LaunchConfiguration('fin_topic_prefix').perform(context)
    fin_topic_suffix = LaunchConfiguration('fin_topic_suffix').perform(context)
    thruster_joy_gain = LaunchConfiguration('thruster_joy_gain').perform(context)

    n_fins = LaunchConfiguration('n_fins').perform(context)
    gain_roll = LaunchConfiguration('gain_roll').perform(context)
    gain_pitch = LaunchConfiguration('gain_pitch').perform(context)
    gain_yaw = LaunchConfiguration('gain_yaw').perform(context)

    nodes = []

    # === Common joystick node ===
    joystick_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick',
        parameters=[{
            'autorepeat_rate': 10.0,
            'dev': f"/dev/input/js{joy_id}"
        }]
    )

    # === Conditional teleop setup ===
    if use_param_file:
        # Case 1: Using parameter file
        teleop_node = Node(
            package='uuv_teleop',
            executable='finned_uuv_teleop.py',
            name='finned_uuv_teleop',
            output='screen',
            parameters=[filename]  # load all params from the YAML file
        )

        nodes.append(GroupAction([
            PushRosNamespace(uuv_name),
            teleop_node,
            joystick_node
        ]))

    else:
        # Case 2: Inline parameter configuration
        teleop_node = Node(
            package='uuv_teleop',
            executable='finned_uuv_teleop.py',
            name='finned_uuv_teleop',
            output='screen',
            parameters=[{
                'axis_thruster': int(axis_thruster),
                'axis_roll': int(axis_roll),
                'axis_pitch': int(axis_pitch),
                'axis_yaw': int(axis_yaw),
                'n_fins': int(n_fins),
                'thruster_joy_gain': float(thruster_joy_gain),
                'thruster_model': {
                    'name': 'proportional',
                    'max_thrust': float(max_thrust),
                    'params': {'gain': float(thruster_rotor_gain)}
                },
                'gain_roll': [float(x) for x in gain_roll.split(',')],
                'gain_pitch': [float(x) for x in gain_pitch.split(',')],
                'gain_yaw': [float(x) for x in gain_yaw.split(',')],
                'thruster_topic': thruster_topic,
                'fin_topic_prefix': fin_topic_prefix,
                'fin_topic_suffix': fin_topic_suffix
            }]
        )

        nodes.append(GroupAction([
            PushRosNamespace(uuv_name),
            teleop_node,
            joystick_node
        ]))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('joy_id', default_value='0'),
        DeclareLaunchArgument('use_param_file', default_value='false'),
        DeclareLaunchArgument('filename', default_value='.'),

        DeclareLaunchArgument('axis_thruster', default_value='1'),
        DeclareLaunchArgument('axis_roll', default_value='0'),
        DeclareLaunchArgument('axis_pitch', default_value='4'),
        DeclareLaunchArgument('axis_yaw', default_value='3'),

        DeclareLaunchArgument('thruster_rotor_gain', default_value='0.0009'),
        DeclareLaunchArgument('max_thrust', default_value='200'),
        DeclareLaunchArgument('thruster_topic', default_value='thrusters/id_0/input'),
        DeclareLaunchArgument('fin_topic_prefix', default_value='fins/'),
        DeclareLaunchArgument('fin_topic_suffix', default_value='/input'),
        DeclareLaunchArgument('thruster_joy_gain', default_value='1.0'),

        DeclareLaunchArgument('n_fins', default_value='4'),
        DeclareLaunchArgument('gain_roll', default_value='1,1,1,1'),
        DeclareLaunchArgument('gain_pitch', default_value='1,1,-1,-1'),
        DeclareLaunchArgument('gain_yaw', default_value='-1,1,1,-1'),

        OpaqueFunction(function=launch_setup)
    ])
