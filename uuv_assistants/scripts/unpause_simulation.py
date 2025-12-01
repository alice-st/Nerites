#!/usr/bin/env python3

# Copyright (c) 2020 The Plankton Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# http://www.apache.org/licenses/LICENSE-2.0

# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import time

from nerites_utils.time import is_sim_time


class UnpauseSimulation(Node):
    """ROS 2 node to unpause Gazebo physics after an optional timeout."""

    def __init__(self):
        super().__init__(
            'unpause_simulation',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        # Read the timeout parameter (defaults to 0)
        self.timeout = 0.0
        if self.has_parameter('timeout'):
            self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
            if self.timeout < 0:
                raise RuntimeError('"timeout" must be a non-negative floating point value')

        self.get_logger().info(f'Waiting {self.timeout:.1f} seconds before unpausing Gazebo physics...')

        if self.timeout > 0:
            time.sleep(self.timeout)

        # Create Gazebo unpause service client
        self.unpause_client = self.create_client(Empty, '/gazebo/unpause_physics')
        self.get_logger().info('Waiting for Gazebo "unpause_physics" service...')

        if not self.unpause_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error('Service /gazebo/unpause_physics not available after 30 seconds.')
            raise RuntimeError('Service /gazebo/unpause_physics not available')

        self.get_logger().info('Service available, sending unpause request...')
        self.unpause()

    def unpause(self):
        """Send unpause request to Gazebo."""
        request = Empty.Request()
        future = self.unpause_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Gazebo simulation unpaused successfully.')
        else:
            self.get_logger().error('Failed to receive a response from Gazebo unpause service.')


def main():
    rclpy.init()
    try:
        # Optional sim time param injection
        sim_time_param = is_sim_time()
        node = UnpauseSimulation()
        node.set_parameters([sim_time_param])  # apply sim_time if needed
        rclpy.shutdown()
    except Exception as e:
        print(f'Exception while unpausing simulation: {e}')
        rclpy.shutdown()


if __name__ == '__main__':
    main()
