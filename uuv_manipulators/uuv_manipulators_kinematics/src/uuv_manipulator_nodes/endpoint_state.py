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

import sys
import argparse
import os

import numpy as np

import rclpy
from rclpy.node import Node

from uuv_manipulators_kinematics import ManipulatorKinematics
from uuv_manipulators_msgs.msg import EndPointState
from geometry_msgs.msg import Vector3


class EndPointStateNode(Node):
    def __init__(self, namespace: str):
        super().__init__('endpoint_state_publisher')

        # Storing the manipulator's namespace
        self._namespace = namespace
        if self._namespace and self._namespace[0] != '/':
            self._namespace = '/' + self._namespace

        # Calling the interface to the kinematics KDL interface
        # NOTE: if you port ManipulatorKinematics to ROS 2 and make it need a node,
        # you can change this to ManipulatorKinematics(self._namespace, node=self)
        self._kinematics = ManipulatorKinematics(self._namespace)

        # Creating a publisher for the end point state
        topic_name = self._namespace + '/endpoint_state'
        self._pub = self.create_publisher(EndPointState, topic_name, 10)

        # Timer at 500 Hz
        self._timer_period = 1.0 / 500.0
        self._timer = self.create_timer(self._timer_period, self._update)

        self.get_logger().info(f'Endpoint state publisher started on {topic_name}')

    def _update(self):
        # EndPointState message instance
        state_msg = EndPointState()

        # Read joint states from the kinematics interface
        # Use dicts directly (so KinChainInterface-style code works correctly)
        q = self._kinematics.joint_angles
        qd = self._kinematics.joint_velocities
        eff = self._kinematics.joint_efforts

        # Storing the pose
        pose = self._kinematics.forward_position_kinematics(q)
        state_msg.pose.position.x = float(pose[0])
        state_msg.pose.position.y = float(pose[1])
        state_msg.pose.position.z = float(pose[2])

        state_msg.pose.orientation.x = float(pose[3])
        state_msg.pose.orientation.y = float(pose[4])
        state_msg.pose.orientation.z = float(pose[5])
        state_msg.pose.orientation.w = float(pose[6])

        # Storing velocity
        # forward_velocity_kinematics(joint_values, joint_velocities)
        vel = self._kinematics.forward_velocity_kinematics(q, qd)
        # PyKDL Twist → geometry_msgs/Vector3
        state_msg.twist.linear.x = float(vel.vel.x())
        state_msg.twist.linear.y = float(vel.vel.y())
        state_msg.twist.linear.z = float(vel.vel.z())
        state_msg.twist.angular.x = float(vel.rot.x())
        state_msg.twist.angular.y = float(vel.rot.y())
        state_msg.twist.angular.z = float(vel.rot.z())

        # Calculating wrench: J(q) * tau
        J = self._kinematics.jacobian(q)
        eff_vec = np.array(list(eff.values()), dtype=float)
        wrench = np.dot(J, eff_vec)  # shape (6,)

        state_msg.wrench.force.x = float(wrench[0])
        state_msg.wrench.force.y = float(wrench[1])
        state_msg.wrench.force.z = float(wrench[2])
        state_msg.wrench.torque.x = float(wrench[3])
        state_msg.wrench.torque.y = float(wrench[4])
        state_msg.wrench.torque.z = float(wrench[5])

        self._pub.publish(state_msg)


def main():
    # Create argument parser
    parser = argparse.ArgumentParser(description='End point state publisher')
    parser.add_argument('--namespace', metavar='NS', type=str)

    # Filter out ROS remapping arguments (like old-style ":=") and script name
    arguments = [a for a in sys.argv[1:] if ':=' not in a]

    args, _ = parser.parse_known_args(arguments)

    if not args.namespace:
        raise RuntimeError('Namespace was not given (use --namespace NS)')

    namespace = args.namespace

    rclpy.init(args=sys.argv)
    node = EndPointStateNode(namespace)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down endpoint_state_publisher')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
