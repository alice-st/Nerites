#!/usr/bin/env python3

# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

import rclpy
from rclpy.node import Node
import numpy as np
from copy import deepcopy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32
from gazebo_msgs.srv import GetWorldProperties, GetModelProperties
from tf_quaternion.transformations import euler_from_quaternion

from nerites_utils.time import is_sim_time


class FootprintsPublisher(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        # Keep track of which models we’re publishing footprints for
        self.vehicle_pub = {}  # {model_name: publisher}
        self.odom_sub = {}     # {model_name: subscription}

        # Gazebo service clients
        self.get_world_props = self.create_client(GetWorldProperties, '/gazebo/get_world_properties')
        self.get_model_props = self.create_client(GetModelProperties, '/gazebo/get_model_properties')

        self.marker = np.array([[0, 0.75], [-0.5, -0.25], [0.5, -0.25]])

        # Wait for services to be ready
        self.get_logger().info("Waiting for /gazebo/get_world_properties...")
        self.get_world_props.wait_for_service()

        self.get_logger().info("Waiting for /gazebo/get_model_properties...")
        self.get_model_props.wait_for_service()

        # Timer to refresh the vehicle list every 10 seconds
        self.create_timer(10.0, self.update_vehicle_list)

    @staticmethod
    def rot(alpha):
        """Return a 2D rotation matrix for angle alpha."""
        return np.array([[np.cos(alpha), -np.sin(alpha)],
                         [np.sin(alpha), np.cos(alpha)]])

    def odometry_callback(self, msg, name):
        """Callback to draw a footprint polygon for the given vehicle."""
        if name not in self.vehicle_pub:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        orientation = np.array([msg.pose.pose.orientation.x,
                                msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z,
                                msg.pose.pose.orientation.w])

        yaw = euler_from_quaternion(orientation)[2]

        # Create the footprint polygon
        new_marker = deepcopy(self.marker)
        points = []
        for i in range(new_marker.shape[0]):
            new_marker[i, :] = np.dot(self.rot(yaw - np.pi / 2), new_marker[i, :])
            new_marker[i, 0] += x
            new_marker[i, 1] += y

            p = Point32()
            p.x = new_marker[i, 0]
            p.y = new_marker[i, 1]
            points.append(p)

        poly_msg = PolygonStamped()
        poly_msg.header.stamp = self.get_clock().now().to_msg()
        poly_msg.header.frame_id = 'world'
        poly_msg.polygon.points = points

        self.vehicle_pub[name].publish(poly_msg)

    def sub_odometry_topic(self, model_name):
        """Subscribe to the Odometry topic for the given model if one exists."""
        # Use ROS 2 topic introspection instead of rostopic
        all_topics = self.get_topic_names_and_types()
        for topic_name, type_list in all_topics:
            if model_name in topic_name and 'nav_msgs/msg/Odometry' in type_list:
                if model_name not in self.odom_sub:
                    self.get_logger().info(f"Subscribing to odometry topic {topic_name} for {model_name}")
                    self.odom_sub[model_name] = self.create_subscription(
                        Odometry,
                        topic_name,
                        lambda msg, name=model_name: self.odometry_callback(msg, name),
                        10
                    )
                return

        self.get_logger().warn(f"No odometry topic found for {model_name}")

    def update_vehicle_list(self):
        """Refresh the list of vehicles from Gazebo and attach odometry subs + footprint publishers."""
        self.get_logger().info("Refreshing vehicle list from Gazebo...")

        reqW = GetWorldProperties.Request()

        future = self.get_world_props.call_async(reqW)
        rclpy.spin_until_future_complete(self, future)
        if not future.result():
            self.get_logger().error("Failed to call /gazebo/get_world_properties")
            return

        world_props = future.result()

        for model in world_props.model_names:
            # Get model properties
            reqM = GetModelProperties.Request()
            reqM.model_name = model

            future_m = self.get_model_props.call_async(reqM)
            rclpy.spin_until_future_complete(self, future_m)
            if not future_m.result():
                self.get_logger().warn(f"Could not get properties for {model}")
                continue

            model_props = future_m.result()

            # Filter out static models and only track new ones
            if not model_props.is_static and model not in self.vehicle_pub:
                self.get_logger().info(f"Found dynamic model: {model}")
                # Subscribe to odometry topic for this model
                self.sub_odometry_topic(model)

                # Create publisher for footprint polygon
                pub = self.create_publisher(PolygonStamped, f'/{model}/footprint', 10)
                self.vehicle_pub[model] = pub


def main():
    rclpy.init()
    try:
        sim_time_param = is_sim_time()
        node = FootprintsPublisher('publish_footprints', parameter_overrides=[sim_time_param])
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down footprint publisher.')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
