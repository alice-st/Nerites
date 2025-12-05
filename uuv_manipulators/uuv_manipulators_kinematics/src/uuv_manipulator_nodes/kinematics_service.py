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

import rclpy
from rclpy.node import Node

from uuv_manipulator_interfaces.arm import ArmInterface
from uuv_manipulators_msgs.msg import EndPointState
from uuv_manipulators_msgs.srv import SolveIK
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class KinematicsService(Node):
    def __init__(self):
        node_name = os.path.splitext(os.path.basename(__file__))[0]
        super().__init__(node_name)

        # Namespace is expected to be /<uuv_name>/<arm_name>/
        ns_tokens = [item for item in self.get_namespace().split('/') if len(item) > 0]
        if len(ns_tokens) != 2:
            raise RuntimeError(
                'The controller must be called in the manipulator namespace '
                '(/<uuv_name>/<arm_name>)'
            )

        self._namespace = ns_tokens[0]
        self._arm_name = ns_tokens[1]

        # Normalize our stored namespace to "/name/"
        if not self._namespace.startswith('/'):
            self._namespace = '/' + self._namespace
        if not self._namespace.endswith('/'):
            self._namespace += '/'

        # The arm interface loads URDF parameters and initializes the KDL chain.
        # IMPORTANT: pass the node and names to the ROS2 ArmInterface.
        self._arm_interface = ArmInterface(
            node=self,
            namespace=self._namespace.strip('/'),
            arm_name=self._arm_name,
            compute_fk_for_all=False
        )

        self._base_link = self._arm_interface.base_link
        self._tip_link = self._arm_interface.tip_link

        # Add a callback function to publish the end effector's state on joint updates
        self._arm_interface.add_callback(
            'joint_states',
            self.on_update_endeffector_state
        )
        # Optionally publish the manipulability index:
        # self._arm_interface.add_callback('joint_states', self.publish_man_index)

        # Publishers (names are relative; node namespace will prefix /<uuv>/<arm>/)
        self._endeffector_state_pub = self.create_publisher(
            EndPointState,
            'endpoint_state',
            10
        )

        self._man_index_pub = self.create_publisher(
            Float64,
            'man_index',
            10
        )

        # Services
        self._services = {}
        self._services['ik'] = self.create_service(
            SolveIK,
            'ik_solver',
            self.solve_ik
        )

        self.get_logger().info(
            f'KinematicsService initialized for {self._namespace}{self._arm_name}'
        )

    # -------------------------------------------------------------------------- #
    #  Properties                                                                #
    # -------------------------------------------------------------------------- #
    @property
    def joint_names(self):
        return self._arm_interface.joint_names

    @property
    def joint_angles(self):
        return self._arm_interface.joint_angles

    @property
    def joint_velocities(self):
        return self._arm_interface.joint_velocities

    @property
    def joint_efforts(self):
        return self._arm_interface.joint_efforts

    @property
    def home(self):
        return self._arm_interface.home

    # -------------------------------------------------------------------------- #
    #  Service callbacks                                                         #
    # -------------------------------------------------------------------------- #
    def solve_ik(self, request, response):
        """Solve inverse kinematics using the ArmInterface IK solver.

        This is a ROS 2-style callback: (request, response) -> response
        The 'response' object is created by rclpy and passed in.
        """
        response.is_valid = False
        response.joints = JointState()

        pos = [
            request.pose.position.x,
            request.pose.position.y,
            request.pose.position.z
        ]
        orientation = [
            request.pose.orientation.x,
            request.pose.orientation.y,
            request.pose.orientation.z,
            request.pose.orientation.w
        ]

        result_ik = self._arm_interface.inverse_kinematics(pos, orientation)

        if result_ik is not None:
            for i, name in zip(range(len(self.joint_names)), self.joint_names):
                response.joints.name.append(name)
                response.joints.position.append(float(result_ik[i]))
            response.is_valid = True

        return response

    # -------------------------------------------------------------------------- #
    #  Helper publishers                                                         #
    # -------------------------------------------------------------------------- #
    def publish_man_index(self):
        w_msg = Float64()
        w_msg.data = float(self._arm_interface.man_index)
        self._man_index_pub.publish(w_msg)

    def on_update_endeffector_state(self):
        state_msg = EndPointState()

        # Pose
        pose = self._arm_interface.endeffector_pose
        state_msg.pose.position.x = float(pose['position'][0])
        state_msg.pose.position.y = float(pose['position'][1])
        state_msg.pose.position.z = float(pose['position'][2])

        state_msg.pose.orientation.x = float(pose['orientation'][0])
        state_msg.pose.orientation.y = float(pose['orientation'][1])
        state_msg.pose.orientation.z = float(pose['orientation'][2])
        state_msg.pose.orientation.w = float(pose['orientation'][3])

        # Twist
        twist = self._arm_interface.endeffector_twist
        state_msg.twist.linear.x = float(twist['linear'][0])
        state_msg.twist.linear.y = float(twist['linear'][1])
        state_msg.twist.linear.z = float(twist['linear'][2])

        state_msg.twist.angular.x = float(twist['angular'][0])
        state_msg.twist.angular.y = float(twist['angular'][1])
        state_msg.twist.angular.z = float(twist['angular'][2])

        # Wrench
        wrench = self._arm_interface.endeffector_wrench
        state_msg.wrench.force.x = float(wrench['force'][0])
        state_msg.wrench.force.y = float(wrench['force'][1])
        state_msg.wrench.force.z = float(wrench['force'][2])

        state_msg.wrench.torque.x = float(wrench['torque'][0])
        state_msg.wrench.torque.y = float(wrench['torque'][1])
        state_msg.wrench.torque.z = float(wrench['torque'][2])

        self._endeffector_state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down KinematicsService')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
