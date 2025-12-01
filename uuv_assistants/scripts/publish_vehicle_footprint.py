#!/usr/bin/env python3

# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

import rclpy
from rclpy.node import Node
from copy import deepcopy
import numpy as np

from tf_quaternion.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import Marker

from nerites_utils.time import is_sim_time


class VehicleFootprint(Node):
    """Publishes vehicle footprint polygons and label markers for RViz."""
    
    MARKER = np.array([[0, 0.75], [-0.5, -0.25], [0.5, -0.25]])
    
    def __init__(self, name, **kwargs):
        super().__init__(name,
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True,
                         **kwargs)

        self.get_logger().info('Generating RViz footprint and labels for 2D visualization')

        # --- Read parameters (with defaults) ---
        self._namespace = self.get_namespace().replace('/', '')

        self._scale_footprint = 10.0
        if self.has_parameter('scale_footprint'):
            scale = self.get_parameter('scale_footprint').get_parameter_value().double_value
            if scale > 0:
                self._scale_footprint = scale
            else:
                self.get_logger().error('scale_footprint must be > 0')

        self.get_logger().info(f'Footprint scale factor = {self._scale_footprint}')

        self._scale_label = 10.0
        if self.has_parameter('scale_label'):  # FIXED param name
            scale = self.get_parameter('scale_label').get_parameter_value().double_value
            if scale > 0:
                self._scale_label = scale
            else:
                self.get_logger().error('scale_label must be > 0')

        self.get_logger().info(f'Label scale factor = {self._scale_label}')

        self._label_x_offset = 60.0
        if self.has_parameter('label_x_offset'):
            self._label_x_offset = self.get_parameter('label_x_offset').get_parameter_value().double_value

        # --- Label Marker setup ---
        self._label_marker = Marker()
        self._label_marker.header.frame_id = 'world'
        self._label_marker.ns = self._namespace
        self._label_marker.type = Marker.TEXT_VIEW_FACING
        self._label_marker.text = self._namespace
        self._label_marker.action = Marker.ADD
        self._label_marker.pose.orientation.w = 1.0  # identity orientation
        self._label_marker.scale.z = self._scale_label  # only Z is used for text markers
        self._label_marker.color.a = 1.0
        self._label_marker.color.g = 1.0  # green

        # --- ROS Interfaces ---
        self._odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self._footprint_pub = self.create_publisher(PolygonStamped, 'footprint', 10)
        self._label_pub = self.create_publisher(Marker, 'label', 10)

    # =========================================================================
    @staticmethod
    def rot(alpha):
        """2D rotation matrix for angle alpha."""
        return np.array([
            [np.cos(alpha), -np.sin(alpha)],
            [np.sin(alpha),  np.cos(alpha)]
        ])

    # =========================================================================
    def odometry_callback(self, msg):
        """Builds a polygon footprint and a label marker from odometry data."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        orientation = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

        yaw = euler_from_quaternion(orientation)[2]

        # --- Build footprint polygon ---
        new_marker = self._scale_footprint * deepcopy(self.MARKER)
        points = []

        for i in range(new_marker.shape[0]):
            new_marker[i, :] = np.dot(self.rot(yaw - np.pi / 2), new_marker[i, :])
            new_marker[i, 0] += x
            new_marker[i, 1] += y

            p = Point32()
            p.x = new_marker[i, 0]
            p.y = new_marker[i, 1]
            points.append(p)

        new_poly = PolygonStamped()
        new_poly.header.stamp = self.get_clock().now().to_msg()  
        new_poly.header.frame_id = 'world'
        new_poly.polygon.points = points
        self._footprint_pub.publish(new_poly)

        # --- Build label marker ---
        self._label_marker.header.stamp = self.get_clock().now().to_msg()
        self._label_marker.pose.position.x = x + self._label_x_offset
        self._label_marker.pose.position.y = y
        self._label_marker.pose.position.z = msg.pose.pose.position.z

        self._label_pub.publish(self._label_marker)


# =============================================================================
def main():
    rclpy.init()
    print('Starting VehicleFootprint node for RViz visualization')

    try:
        sim_time_param = is_sim_time()
        node = VehicleFootprint('generate_vehicle_footprint', parameter_overrides=[sim_time_param])
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('VehicleFootprint node interrupted')
    finally:
        rclpy.shutdown()
        print('VehicleFootprint node shut down cleanly')


# =============================================================================
if __name__ == '__main__':
    main()
