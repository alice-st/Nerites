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
from copy import deepcopy

import rclpy
from rclpy.node import Node

from uuv_manipulator_interfaces import ArmInterface
from uuv_PID import PIDRegulator
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


class JointPositionController(Node):
    def __init__(self):
        # Node name (similar to ROS1 filename-based)
        node_name = os.path.splitext(os.path.basename(__file__))[0]
        super().__init__(
            node_name,
            automatically_declare_parameters_from_overrides=True
        )

        # --- NEW: derive namespace and arm name like in KinematicsService ----
        # Namespace is expected to be /<uuv_name>/<arm_name>/
        ns_tokens = [item for item in self.get_namespace().split('/') if len(item) > 0]
        if len(ns_tokens) != 2:
            raise RuntimeError(
                'JointPositionController must be called in the manipulator namespace '
                '(/<uuv_name>/<arm_name>)'
            )

        self._namespace = ns_tokens[0]
        self._arm_name = ns_tokens[1]

        # Normalize stored namespace to "/name/"
        if not self._namespace.startswith('/'):
            self._namespace = '/' + self._namespace
        if not self._namespace.endswith('/'):
            self._namespace += '/'

        # ---------------------------------------------------------------------
        # Initializing arm interface  (ROS 2: pass node + names)
        # ---------------------------------------------------------------------
        self._arm_interface = ArmInterface(
            node=self,
            namespace=self._namespace.strip('/'),
            arm_name=self._arm_name,
            compute_fk_for_all=False
        )

        # PID controllers
        self._controllers = dict()
        # Current reference for each joint
        self._reference_pos = dict()
        # Output command topics
        self._command_topics = dict()
        # Axes mapping
        self._axes = dict()
        # Axes gain values
        self._axes_gain = dict()

        # Parameters ---------------------------------------------------------------
        if not self.has_parameter('deadman_button'):
            self.declare_parameter('deadman_button', 5)
        if not self.has_parameter('exclusion_buttons'):
            self.declare_parameter('exclusion_buttons', [4])
        if not self.has_parameter('home_button'):
            self.declare_parameter('home_button', 7)

        # Default for the RB button of the XBox 360 controller
        self._deadman_button = int(self.get_parameter('deadman_button').value)

        # If these buttons are pressed, the arm will not move
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

        # Default for the start button of the XBox 360 controller
        self._home_button = int(self.get_parameter('home_button').value)

        # Controller configuration (required)

        params = self.get_parameters_by_prefix('controller_config')

        if not params:
            raise RuntimeError('Parameter "controller_config" is required but not set')

        controller_config = {}
        for key, param in params.items():
            # key is like "azimuth.joint_input_axis" or "wrist_joint.controller.p"
            parts = key.split('.')
            if len(parts) < 2:
                continue

            joint_name = parts[0]
            rest = parts[1:]

            current = controller_config.setdefault(joint_name, {})

            if len(rest) == 1:
                # e.g. "axis_gain"
                current[rest[0]] = param.value
            elif len(rest) == 2:
                # e.g. "controller.p"
                sub = current.setdefault(rest[0], {})
                sub[rest[1]] = param.value
            else:
                # General case with deeper nesting, just in case
                sub = current
                for p in rest[:-1]:
                    sub = sub.setdefault(p, {})
                sub[rest[-1]] = param.value

        self._controller_config = controller_config


        # Last joystick update timestamp
        self._last_joy_update = self._now()

        # Joystick topic subscriber
        self._joy_sub = self.create_subscription(
            Joy,
            'joy',
            self._joy_callback,
            10
        )

        # Reading the controller configuration --------------------------------------
        for joint in self._arm_interface.joint_names:
            for tag in controller_config:
                if tag in joint:
                    try:
                        cfg = controller_config[tag]

                        # Read the controller parameters
                        p = cfg['controller']['p']
                        i = cfg['controller']['i']
                        d = cfg['controller']['d']

                        self._controllers[joint] = PIDRegulator(p, i, d, 1000)

                        topic_name = cfg['topic']
                        self._command_topics[joint] = self.create_publisher(
                            Float64,
                            topic_name,
                            10
                        )

                        self._axes[joint] = cfg['joint_input_axis']
                        self._axes_gain[joint] = cfg['axis_gain']

                        # Starting reference = home position from robot parameters
                        self._reference_pos[joint] = deepcopy(self._arm_interface.home[joint])

                    except Exception as e:
                        raise RuntimeError(
                            f'Error while trying to setup controller for joint <{joint}>: {e}'
                        )

        # Timer for control loop (50 Hz, same as original rospy.Rate(50))
        self._control_rate = 50.0
        self._timer = self.create_timer(1.0 / self._control_rate, self._update)

        self.get_logger().info(
            f'JointPositionController node started for {self._namespace}{self._arm_name}'
        )

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------

    def _now(self) -> float:
        """Return current time in seconds as float (similar to rospy.get_time())."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _check_joint_limits(self, value, joint):
        """Check the joint position and maintain it within joint limits."""
        output = value
        if self._arm_interface.joint_limits[joint] is not None:
            lower = self._arm_interface.joint_limits[joint]['lower']
            upper = self._arm_interface.joint_limits[joint]['upper']
            if output < lower:
                output = lower
            if output > upper:
                output = upper
        return output

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def _update(self):
        """Control loop: compute PID output and publish commands."""
        pos = self._arm_interface.joint_angles
        now = self._now()
        for joint in pos:
            error = self._reference_pos[joint] - pos[joint]
            u = self._controllers[joint].regulate(error, now)
            msg = Float64()
            msg.data = u
            self._command_topics[joint].publish(msg)

    def _joy_callback(self, joy_msg: Joy):
        try:
            # If deadman button is not pressed, do nothing
            if self._deadman_button != -1:
                if self._deadman_button >= len(joy_msg.buttons):
                    self.get_logger().warn(
                        f'Deadman button index {self._deadman_button} out of range'
                    )
                    return
                if not joy_msg.buttons[self._deadman_button]:
                    return

            # If any exclusion buttons are pressed, do nothing
            for n in self._exclusion_buttons:
                if n < len(joy_msg.buttons) and joy_msg.buttons[n] == 1:
                    return

            # Home button -> go to home configuration
            if self._home_button < len(joy_msg.buttons) and joy_msg.buttons[self._home_button]:
                self._reference_pos = deepcopy(self._arm_interface.home)
                self._last_joy_update = self._now()
            else:
                # Parse joystick input to set joint angle reference
                for joint in self._arm_interface.joint_names:
                    axis_idx = self._axes[joint]
                    if axis_idx >= len(joy_msg.axes):
                        self.get_logger().warn(
                            f'Axis index {axis_idx} out of range for joint {joint}'
                        )
                        continue

                    axis_val = joy_msg.axes[axis_idx]
                    if abs(axis_val) < 0.8:
                        continue
                    else:
                        self._reference_pos[joint] += self._axes_gain[joint] * axis_val
                        # Check joint limits
                        self._reference_pos[joint] = self._check_joint_limits(
                            self._reference_pos[joint], joint
                        )

                self._last_joy_update = self._now()

        except Exception as e:
            self.get_logger().error(
                f'Error during joy parsing, message={e}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = JointPositionController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.get_logger().info('Shutting down JointPositionController node')
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
