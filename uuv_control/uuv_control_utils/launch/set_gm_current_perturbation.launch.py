"""
This script configures the water current model in the UUV Simulator via ROS 2 services.

It sets parameters for one of three current components:
- 'velocity'   : Controls the current speed (m/s).
- 'horz_angle' : Controls the horizontal direction of the current (heading in degrees).
- 'vert_angle' : Controls the vertical inclination of the current (tilt in degrees).

The script reads parameters like mean, min, max, noise, and mu from ROS 2,
validates them, and calls the appropriate Gazebo service
(/hydrodynamics/set_current_<component>_model) to update the simulation.

Typical usage:
    ros2 launch uuv_control_utils set_gm_current_perturbation.launch.py uuv_name:=rexrov component:=horz_angle

"""


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ------------------------
    # Declare launch arguments
    # ------------------------
    args = [
        DeclareLaunchArgument('component'),
        DeclareLaunchArgument('mean', default_value='0.0'),
        DeclareLaunchArgument('min', default_value='0.0'),
        DeclareLaunchArgument('max', default_value='0.0'),
        DeclareLaunchArgument('noise', default_value='0.0'),
        DeclareLaunchArgument('mu', default_value='0.0'),
    ]

    # ------------------------
    # Define the node
    # ------------------------
    gm_current_node = Node(
        package='uuv_control_utils',
        executable='set_gm_current_perturbation.py',
        name='set_gm_current_perturbation',
        output='screen',
        parameters=[{
            'component': LaunchConfiguration('component'),
            'mean': LaunchConfiguration('mean'),
            'min': LaunchConfiguration('min'),
            'max': LaunchConfiguration('max'),
            'noise': LaunchConfiguration('noise'),
            'mu': LaunchConfiguration('mu')
        }]
    )

    # ------------------------
    # Return launch description
    # ------------------------
    return LaunchDescription(args + [gm_current_node])
