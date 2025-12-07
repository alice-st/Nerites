import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    actions = []

    output_dir_path = os.path.join(get_package_share_directory('eca_a9_control'), 'config')

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('uuv_auv_control_allocator'),
                             'launch', 'start_control_allocator.launch.py')
            ]),
            launch_arguments={
                'uuv_name': 'eca_a9',
                'output_dir': output_dir_path,
                'base_link': 'base_link',
                'max_thrust': '700',
                'thruster_conversion_fcn': 'proportional',
                'thruster_gain': '0.000049',
                'fluid_density': '1028.0',
                'lift_coefficient': '3.0',
                'fin_area': '0.04155',
                'fin_lower_joint_limit': '-1.4',
                'fin_upper_joint_limit': '1.4',
                'timeout': '-1',
                'update_rate': '10'
            }.items()
        )
    )

    return actions

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
