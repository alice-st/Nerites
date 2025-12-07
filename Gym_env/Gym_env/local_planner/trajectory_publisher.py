#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Quaternion
from uuv_control_msgs.msg import TrajectoryPoint
from plankton_utils.time import time_in_float_sec

# Import the same LocalPlanner used by DPControllerBase
from .dp_controller_local_planner import DPControllerLocalPlanner as LocalPlanner
# If you have world NED->ENU TF available, pass it the same way DPControllerBase does.

class ReferenceOnlyPublisher(Node):
    def __init__(self):
        super().__init__('reference_only_publisher',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        # Config flags (match what you normally use)
        self._use_stamped_poses_only = self.declare_parameter(
            'use_stamped_poses_only', False).value
        self._planner_full_dof = self.declare_parameter(
            'planner_full_dof', False).value

        # Build the Local Planner — this will also create the waypoint/trajectory services
        self._local_planner = LocalPlanner(
            self,
            full_dof=self._planner_full_dof,
            stamped_pose_only=self._use_stamped_poses_only,
            thrusters_only=True,                # irrelevant here, but required by ctor
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),
            tf_trans_world_ned_to_enu=None      # or your transform func
        )

        # Vehicle pose cache (what planner needs for interpolation)
        self._pos = np.zeros(3)
        self._quat = np.array([0.0, 0.0, 0.0, 1.0])

        # I/O
        self._ref_pub = self.create_publisher(TrajectoryPoint, 'reference', 10)
        self._odom_sub = self.create_subscription(Odometry, 'odom', self._on_odom, 10)

        # Timer to publish reference (e.g., 20 Hz)
        self._timer = self.create_timer(0.05, self._tick)

        self.get_logger().info('Reference-only waypoint interpolator is up. '
                               'Call the service "init_waypoints_from_file" to load a path.')

    def _on_odom(self, msg: Odometry):
        self._pos = np.array([msg.pose.pose.position.x,
                              msg.pose.pose.position.y,
                              msg.pose.pose.position.z], dtype=float)
        q = msg.pose.pose.orientation
        self._quat = np.array([q.x, q.y, q.z, q.w], dtype=float)

    def _tick(self):
        # Keep planner updated with the latest vehicle pose
        self._local_planner.update_vehicle_pose(self._pos, self._quat)

        # Interpolate current trajectory point at simulation time "now"
        t_now = time_in_float_sec(self.get_clock().now())
        ref = self._local_planner.interpolate(t_now)
        if ref is None:
            return

        # Publish as TrajectoryPoint (same shape DPControllerBase would)
        msg = TrajectoryPoint()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._local_planner.inertial_frame_id

        msg.pose.position = Vector3(x=ref.p[0], y=ref.p[1], z=ref.p[2])
        msg.pose.orientation = Quaternion(x=ref.q[0], y=ref.q[1], z=ref.q[2], w=ref.q[3])

        msg.velocity.linear = Vector3(x=ref.v[0], y=ref.v[1], z=ref.v[2])
        msg.velocity.angular = Vector3(x=ref.w[0], y=ref.w[1], z=ref.w[2])

        msg.acceleration.linear = Vector3(x=ref.a[0], y=ref.a[1], z=ref.a[2])
        msg.acceleration.angular = Vector3(x=ref.alpha[0], y=ref.alpha[1], z=ref.alpha[2])

        self._ref_pub.publish(msg)

def main():
    rclpy.init()
    node = ReferenceOnlyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
