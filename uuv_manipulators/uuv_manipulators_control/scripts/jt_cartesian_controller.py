#!/usr/bin/env python3
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

import numpy as np
import PyKDL

import rclpy
from rclpy.node import Node

from uuv_manipulators_control import CartesianController


class JTCartesianController(Node, CartesianController):
    """
    Jacobian transpose controller (ROS 2 Foxy)
    """

    LABEL = 'Jacobian transpose cartesian controller'

    def __init__(self):
        # Node name based on filename, same idea as ROS1 version
        node_name = os.path.splitext(os.path.basename(__file__))[0]
        Node.__init__(self, node_name)
        CartesianController.__init__(self)

        # Retrieve the controller parameters from the parameter server
        self.declare_parameter('Kd', None)
        self.declare_parameter('Kp', None)

        self._Kd = self.get_parameter('Kd').value
        self._Kp = self.get_parameter('Kp').value

        if self._Kd is None:
            raise RuntimeError('Kd gain vector not available (parameter "Kd" not set)')
        if self._Kp is None:
            raise RuntimeError('Kp gain vector not available (parameter "Kp" not set)')

        self.get_logger().info('Kp={}'.format(self._Kp))
        self.get_logger().info('Kd={}'.format(self._Kd))

        # Initialization flag, to wait the end-effector get to the home position
        self._is_init = False

        # Control loop timer (you can tune the frequency if needed)
        self._control_rate = 100.0  # Hz (example)
        self._timer = self.create_timer(1.0 / self._control_rate, self._update)

    def _update(self):
        # Leave if command is not valid
        if self._last_goal is None:
            return

        # Calculate the goal pose
        goal = self._get_goal()

        # End-effector's pose
        ee_pose = self._arm_interface.get_ee_pose_as_frame()
        # End-effector's velocity
        ee_vel = self._arm_interface.get_ee_vel_as_kdl_twist()

        # Calculate pose error
        error = PyKDL.diff(goal, ee_pose)

        # End-effector wrench to achieve target
        wrench = np.matrix(np.zeros(6)).T
        for i in range(len(wrench)):
            wrench[i] = -(self._Kp[i] * error[i] + self._Kd[i] * ee_vel[i])

        # Compute jacobian transpose
        JT = self._arm_interface.jacobian_transpose()
        # Calculate the torques for the joints
        tau = JT * wrench
        # Store current pose target
        self._last_goal = goal

        self.publish_goal()
        self.publish_joint_efforts(tau)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = JTCartesianController()
        node.get_logger().info(f'Starting [{node.get_name()}] node')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.get_logger().info(f'Shutting down [{node.get_name()}] node')
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
