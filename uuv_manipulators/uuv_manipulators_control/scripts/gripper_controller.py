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

from uuv_manipulator_interfaces import GripperInterface
from uuv_manipulators_msgs.msg import EndeffectorCommand  # unused but kept for compatibility
from sensor_msgs.msg import Joy
from uuv_PID import PIDRegulator


class GripperController(Node):

    def __init__(self):
        # Base node name (launch can still remap with __node:=gripper_controller_oberon4)
        node_name = os.path.splitext(os.path.basename(__file__))[0]
        super().__init__(
            node_name,
            automatically_declare_parameters_from_overrides=True
        )

        # ------------------------------------------------------------------
        # Namespace parsing: expect /<uuv_name>/<arm_name>/
        # ------------------------------------------------------------------
        ns_tokens = [item for item in self.get_namespace().split('/') if len(item) > 0]
        if len(ns_tokens) != 2:
            raise RuntimeError(
                'GripperController must be launched in the manipulator namespace '
                '(/<uuv_name>/<arm_name>)'
            )

        self._namespace = ns_tokens[0]   # e.g. 'rexrov'
        self._arm_name = ns_tokens[1]    # e.g. 'oberon4'

        # Normalize to "/name/"
        if not self._namespace.startswith('/'):
            self._namespace = '/' + self._namespace
        if not self._namespace.endswith('/'):
            self._namespace += '/'

        # ------------------------------------------------------------------
        # Initializing the gripper interface for this node + namespace
        # ------------------------------------------------------------------
        self._gripper = GripperInterface(
            node=self,
            namespace=self._namespace.strip('/'),
            arm_name=self._arm_name,
        )

        # Timeout for input inactivity (currently unused, kept for compatibility)
        self._timeout = 0.01

        # ---------------------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------------------
        if not self.has_parameter('deadman_button'):
            self.declare_parameter('deadman_button', 5)
        if not self.has_parameter('open_button'):
            self.declare_parameter('open_button', 1)
        if not self.has_parameter('close_button'):
            self.declare_parameter('close_button', 2)
        if not self.has_parameter('exclusion_buttons'):
            self.declare_parameter('exclusion_buttons', [4])
        if not self.has_parameter('joy_gain'):
            self.declare_parameter('joy_gain', 0.1)
        if not self.has_parameter('publish_rate'):
            self.declare_parameter('publish_rate', 100)
        if not self.has_parameter('kp'):
            self.declare_parameter('kp', 100)
        if not self.has_parameter('ki'):
            self.declare_parameter('ki', 0.0)
        if not self.has_parameter('kd'):
            self.declare_parameter('kd', 0.0)

        # Default - B button from XBox 360 controller
        self._open_button = int(self.get_parameter('open_button').value)

        # Default - X button from XBox 360 controller
        self._close_button = int(self.get_parameter('close_button').value)

        # Default - RB button from XBox 360 controller
        self._deadman_button = int(self.get_parameter('deadman_button').value)

        # Exclusion buttons
        self._exclusion_buttons = self.get_parameter('exclusion_buttons').value

        if isinstance(self._exclusion_buttons, (float, int)):
            self._exclusion_buttons = [int(self._exclusion_buttons)]
        elif isinstance(self._exclusion_buttons, list):
            for n in self._exclusion_buttons:
                if not isinstance(n, (float, int)):
                    raise RuntimeError(
                        'Exclusion buttons must be an integer index to the joystick button'
                    )
            self._exclusion_buttons = [int(n) for n in self._exclusion_buttons]
        else:
            self._exclusion_buttons = []

        # Joystick gain
        self._joy_gain = float(self.get_parameter('joy_gain').value)

        # Publish rate
        self._publish_rate = int(self.get_parameter('publish_rate').value)

        if self._publish_rate <= 0:
            raise RuntimeError('Invalid negative publish rate')

        self._kp = float(self.get_parameter('kp').value)
        self._ki = float(self.get_parameter('ki').value)
        self._kd = float(self.get_parameter('kd').value)

        # ---------------------------------------------------------------------
        # Gripper / PID setup
        # ---------------------------------------------------------------------

        self._pos_goal = self._gripper.closed_pos
        self._ratio_goal = 0.0

        self._controllers = dict()
        self._controllers[self._gripper.control_joint] = PIDRegulator(
            self._kp, self._ki, self._kd, self._publish_rate
        )
        self._controllers[self._gripper.mimic_joint] = PIDRegulator(
            self._kp, self._ki, self._kd, self._publish_rate
        )

        if self._gripper.control_joint is None:
            raise RuntimeError('Gripper cannot be controlled')

        self._limits = self._gripper.control_joint_limits

        # ---------------------------------------------------------------------
        # ROS 2 interfaces
        # ---------------------------------------------------------------------

        # Joy subscriber
        self._joy_sub = self.create_subscription(
            Joy,
            'joy',
            self._joy_callback,
            10
        )

        # Timer to replace the rospy.Rate loop
        self._timer = self.create_timer(
            1.0 / float(self._publish_rate),
            self._update
        )

        self.get_logger().info(
            f'GripperController node started for {self._namespace}{self._arm_name}'
        )

    # -------------------------------------------------------------------------
    # Internal methods
    # -------------------------------------------------------------------------

    def _now(self) -> float:
        """Return current time in seconds as float (similar to rospy.get_time())."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _update(self):
        # Compute the necessary effort for the requested joint position
        self._pos_goal = self._ratio_goal * (
            self._gripper.fully_open_pos - self._gripper.closed_pos
        ) + self._gripper.closed_pos

        now = self._now()

        # Set the torque for the control joint
        error = self._pos_goal - self._gripper.control_joint_position
        torque = self._controllers[self._gripper.control_joint].regulate(error, now)
        self._gripper.set_controller_command(self._gripper.control_joint, torque)

        # Set the torque for the mimic joint
        error = self._pos_goal - self._gripper.mimic_joint_position
        torque = self._controllers[self._gripper.mimic_joint].regulate(error, now)
        self._gripper.set_controller_command(self._gripper.mimic_joint, torque)

        if self._gripper.is_parallel:
            # Placeholder: in case special handling is needed for parallel grippers
            pass

    def _joy_callback(self, joy_msg: Joy):
        try:
            if self._deadman_button > -1:
                if self._deadman_button >= len(joy_msg.buttons):
                    self.get_logger().warn(
                        f'Deadman button index {self._deadman_button} out of range for joystick'
                    )
                    return
                if not joy_msg.buttons[self._deadman_button]:
                    return

            for n in self._exclusion_buttons:
                if n < len(joy_msg.buttons) and joy_msg.buttons[n] == 1:
                    return

            # Ensure button indices are valid
            if (self._open_button >= len(joy_msg.buttons) or
                    self._close_button >= len(joy_msg.buttons)):
                self.get_logger().warn(
                    'Open/close button index out of range for joystick'
                )
                return

            self._ratio_goal += self._joy_gain * (
                joy_msg.buttons[self._open_button] - joy_msg.buttons[self._close_button]
            )

            if self._ratio_goal < 0.0:
                self._ratio_goal = 0.0
            if self._ratio_goal > 1.0:
                self._ratio_goal = 1.0

        except Exception as e:
            self.get_logger().error(
                'Error occurred while parsing joystick input, check if the '
                f'joy_id corresponds to the joystick being used. message={str(e)}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = GripperController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.get_logger().info('Shutting down gripper controller node')
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
