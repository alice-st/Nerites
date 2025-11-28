from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Declare launch arguments
    uuv_name_arg = DeclareLaunchArgument('uuv_name')
    scale_footprint_arg = DeclareLaunchArgument('scale_footprint', default_value='10')
    scale_label_arg = DeclareLaunchArgument('scale_label', default_value='10')
    label_x_offset_arg = DeclareLaunchArgument('label_x_offset', default_value='60')
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value='pose_gt')

    # Define the node
    publish_footprints_node = Node(
        package='uuv_assistants',
        executable='publish_vehicle_footprint.py',
        name='publish_footprints',
        output='screen',
        remappings=[('odom', LaunchConfiguration('odom_topic'))],
        parameters=[{
            'scale_footprint': LaunchConfiguration('scale_footprint'),
            'scale_label': LaunchConfiguration('scale_label'),
            'label_x_offset': LaunchConfiguration('label_x_offset')
        }]
    )

    # Group actions for namespace
    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('uuv_name')),
        publish_footprints_node
    ])

    # Build the launch description
    return LaunchDescription([
        uuv_name_arg,
        scale_footprint_arg,
        scale_label_arg,
        label_x_offset_arg,
        odom_topic_arg,
        group
    ])
