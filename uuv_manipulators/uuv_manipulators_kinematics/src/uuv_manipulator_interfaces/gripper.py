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

import time
from typing import Dict, Optional

import numpy as np
import PyKDL

import rclpy
from rclpy.node import Node

from uuv_manipulator_interfaces.kin_chain import KinChainInterface
from uuv_manipulators_msgs.msg import EndeffectorState  # , GripperState (if available)
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


class GripperInterface(object):
    TYPE = ['no_gripper', 'parallel', 'jaw']
    STATE = ['ready', 'disabled', 'moving']

    def __init__(self, node: Node, namespace: Optional[str] = None, arm_name: Optional[str] = None):
        """
        :param node: rclpy Node that owns parameters, pubs/subs, and timers.
        :param namespace: manipulator namespace (optional; derived from node namespace if None)
        :param arm_name: arm name (optional; derived from node namespace if None)
        """
        self._node = node

        # Published topics
        self._pubTopics: Dict[str, object] = {}
        # Controller input topics
        self._pubControllers: Dict[str, object] = {}
        # Subscribed topics
        self._subTopics: Dict[str, object] = {}

        # Derive namespace and arm_name from node namespace if not given
        if None in (namespace, arm_name):
            ns_tokens = [item for item in node.get_namespace().split('/') if item]
            if len(ns_tokens) != 2:
                raise RuntimeError('The controller must be called inside the manipulator namespace')
            self._namespace = ns_tokens[0]
            self._arm_name = ns_tokens[1]
        else:
            self._namespace = namespace
            self._arm_name = arm_name

        # Normalize namespace to '/name/'
        if not self._namespace.startswith('/'):
            self._namespace = '/' + self._namespace
        if not self._namespace.endswith('/'):
            self._namespace += '/'

        # ---------------------------------------------------------------------
        # Retrieve gripper configuration
        #
        # ROS 2 YAML like:
        #
        # gripper_controller_oberon4:
        #   ros__parameters:
        #     gripper:
        #       type: jaw
        #       base: gripper_base
        #       ...
        #
        # produces parameters:
        #   gripper.type
        #   gripper.base
        #   gripper.links
        #   gripper.groups.left_finger.ee
        #   ...
        #
        # So we first try to reconstruct a nested dict from parameters with
        # prefix 'gripper', and fall back to a single 'gripper' parameter if
        # someone set it explicitly.
        # ---------------------------------------------------------------------
        self._gripper_config = self._load_gripper_config()

        if not self._gripper_config:
            raise RuntimeError(
                'Gripper configuration not given (parameters with prefix "gripper" are not set)'
            )

        # Retrieve gripper type
        if 'type' not in self._gripper_config:
            raise RuntimeError('Gripper type not defined')

        if self._gripper_config['type'] not in self.TYPE:
            raise RuntimeError('Gripper type not supported')

        self._gripper_type = self._gripper_config['type']

        self._closed_limit = self._gripper_config.get('closed_limit', 'upper')
        self._full_open_limit = self._gripper_config.get('full_open_limit', 'upper')

        self._node.get_logger().info('Gripper type=' + self._gripper_type)

        # Chain groups
        self._groups: Dict[str, KinChainInterface] = {}

        # Loading the kinematic chain for each finger
        for group in self._gripper_config['groups']:
            chain = self._gripper_config['groups'][group]
            self._groups[group] = KinChainInterface(
                node=self._node,               
                name=group,
                base=self._gripper_config['base'],
                ee_link=chain['ee'],
                namespace=self._namespace,
                arm_name=self._arm_name,
                compute_fk_for_all=False
            )

        # Subscribe to the joint states topic
        joint_states_topic = self._namespace + 'joint_states'
        self._subTopics['joint_states'] = self._node.create_subscription(
            JointState,
            joint_states_topic,
            self._on_joint_states,
            1
        )

        # Publish end-effector state
        ee_state_topic = self._namespace + self._arm_name + '/ee_state'
        self._pubTopics['state'] = self._node.create_publisher(
            EndeffectorState,
            ee_state_topic,
            1
        )

        # Set gripper state
        self._gripper_state = 'ready'

        # Retrieving the name of the control joint
        self._control_joint = None
        self._control_joint_group = None

        self._mimic_joint = None
        self._mimic_joint_group = None

        if 'control_joint' in self._gripper_config:
            # Correct the name including the namespace in TF
            for name in self._groups:
                for joint in self._groups[name].joint_names:
                    if self._gripper_config['control_joint'] in joint:
                        self._control_joint = joint
                        self._control_joint_group = name
                    if 'mimic_joint' in self._gripper_config and \
                            self._gripper_config['mimic_joint'] in joint:
                        self._mimic_joint = joint
                        self._mimic_joint_group = name

            if self._control_joint_group is not None and self._control_joint is not None:
                self._node.get_logger().info(
                    f'{self._control_joint_group} - {self._control_joint}'
                )

        # Create controller publishers for each joint
        for joint in self.get_joint_names():
            topic = self._namespace + joint + '/controller/command'
            self._pubControllers[joint] = self._node.create_publisher(Float64, topic, 1)

        # Publishing rate for state topics
        self._publish_rate = 50.0
        # Set timer to regularly publish the state
        self._publish_state_timer = self._node.create_timer(
            1.0 / self._publish_rate,
            self._publish_endeffector_state
        )

    # -------------------------------------------------------------------------
    # Internal helpers
    # -------------------------------------------------------------------------

    def _build_gripper_config_from_params(self, params: Dict[str, rclpy.parameter.Parameter]) -> Dict:
        """Convert ROS 2 parameters with prefix 'gripper.' into a nested dict.

        Example keys from get_parameters_by_prefix('gripper'):
          'type'
          'base'
          'links'
          'groups.left_finger.ee'

        become:
        {
          'type': 'jaw',
          'base': 'gripper_base',
          'links': [...],
          'groups': {
            'left_finger': {
              'ee': 'jaw_1',
              ...
            },
            ...
          }
        }
        """
        cfg: Dict[str, object] = {}
        for short_name, param in params.items():
            parts = short_name.split('.')
            cur = cfg
            for part in parts[:-1]:
                if part not in cur or not isinstance(cur[part], dict):
                    cur[part] = {}
                cur = cur[part]
            cur[parts[-1]] = param.value
        return cfg

    def _load_gripper_config(self) -> Dict:
        """Load gripper configuration from ROS 2 parameters."""
        # First try nested ROS 2 parameters under the 'gripper' prefix
        params_with_prefix = self._node.get_parameters_by_prefix('gripper')
        if params_with_prefix:
            return self._build_gripper_config_from_params(params_with_prefix)

        # Fallback: single 'gripper' parameter (dict or YAML string)
        if not self._node.has_parameter('gripper'):
            self._node.declare_parameter('gripper', None)
        param = self._node.get_parameter('gripper')

        # If it's a string, user might have passed raw YAML; leave that alone.
        if param.type_ == param.Type.STRING:
            return param.get_parameter_value().string_value
        else:
            return param.value

    # -------------------------------------------------------------------------
    # Cleanup
    # -------------------------------------------------------------------------

    def __del__(self):
        """Class destructor."""
        # Destroy timer
        try:
            if hasattr(self, '_publish_state_timer') and self._publish_state_timer is not None:
                self._publish_state_timer.cancel()
                self._node.destroy_timer(self._publish_state_timer)
        except Exception:
            pass

        # Destroy subscriptions
        for topic in list(self._subTopics.keys()):
            sub = self._subTopics[topic]
            try:
                self._node.destroy_subscription(sub)
            except Exception:
                pass

        # Destroy publishers
        for topic in list(self._pubTopics.keys()):
            pub = self._pubTopics[topic]
            try:
                self._node.destroy_publisher(pub)
            except Exception:
                pass

        for joint in list(self._pubControllers.keys()):
            pub = self._pubControllers[joint]
            try:
                self._node.destroy_publisher(pub)
            except Exception:
                pass

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def full_namespace(self):
        return self._namespace + self._arm_name

    @property
    def is_fully_open(self):
        if self._control_joint is None:
            return None
        return self.control_joint_position == self.control_joint_limits[self._full_open_limit]

    @property
    def is_closed(self):
        if self._control_joint is None:
            return None
        return self.control_joint_position == self.control_joint_limits[self._closed_limit]

    @property
    def closed_pos(self):
        if self._control_joint is None:
            return None
        return self.control_joint_limits[self._closed_limit]

    @property
    def fully_open_pos(self):
        if self._control_joint is None:
            return None
        return self.control_joint_limits[self._full_open_limit]

    @property
    def is_moving(self):
        if self.control_joint is None:
            return None
        return self.control_joint_velocity > 0.001

    @property
    def is_ready(self):
        return self._gripper_state == 'ready'

    @property
    def is_disabled(self):
        return self._gripper_state == 'disabled'

    @property
    def is_parallel(self):
        return self._gripper_type == 'parallel'

    @property
    def is_jaw(self):
        return self._gripper_type == 'jaw'

    @property
    def control_joint(self):
        return self._control_joint

    @property
    def control_joint_group(self):
        return self._control_joint_group

    @property
    def control_joint_position(self):
        if None in (self._control_joint, self._control_joint_group):
            return None
        return self._groups[self._control_joint_group].joint_angles[self._control_joint]

    @property
    def control_joint_velocity(self):
        if None in (self._control_joint, self._control_joint_group):
            return None
        return self._groups[self._control_joint_group].joint_velocities[self._control_joint]

    @property
    def control_joint_effort(self):
        if None in (self._control_joint, self._control_joint_group):
            return None
        return self._groups[self._control_joint_group].joint_efforts[self._control_joint]

    @property
    def control_joint_limits(self):
        if None in (self._control_joint, self._control_joint_group):
            return None
        return self._groups[self._control_joint_group].joint_limits[self._control_joint]

    @property
    def control_joint_pos_ratio(self):
        return self.get_position_ratio(self.control_joint_position)

    @property
    def mimic_joint(self):
        return self._mimic_joint

    @property
    def mimic_joint_group(self):
        return self._mimic_joint_group

    @property
    def mimic_joint_position(self):
        if None in (self._mimic_joint, self._mimic_joint_group):
            return None
        return self._groups[self._mimic_joint_group].joint_angles[self._mimic_joint]

    @property
    def mimic_joint_velocity(self):
        if None in (self._mimic_joint, self._mimic_joint_group):
            return None
        return self._groups[self._mimic_joint_group].joint_velocities[self._mimic_joint]

    @property
    def mimic_joint_effort(self):
        if None in (self._mimic_joint, self._mimic_joint_group):
            return None
        return self._groups[self._mimic_joint_group].joint_efforts[self._mimic_joint]

    @property
    def mimic_joint_limits(self):
        if None in (self._mimic_joint, self._mimic_joint_group):
            return None
        return self._groups[self._mimic_joint_group].joint_limits[self._mimic_joint]

    @property
    def mimic_joint_pos_ratio(self):
        return self.get_position_ratio(self.mimic_joint_position)

    # -------------------------------------------------------------------------
    # Callbacks and helpers
    # -------------------------------------------------------------------------

    def _on_joint_states(self, msg: JointState):
        for name in self._groups:
            # Store the joint states
            self._groups[name].update_joint_states(msg)

    def _publish_endeffector_state(self):
        msg = EndeffectorState()
        msg.stamp = self._node.get_clock().now().to_msg()
        msg.state = self._gripper_state.lower()
        self._pubTopics['state'].publish(msg)

    def get_position_ratio(self, pos):
        return (pos - self.closed_pos) / (self.fully_open_pos - self.closed_pos)

    def get_type(self):
        return self._gripper_type

    def get_joint_names(self, group=None):
        if group is None:
            joint_names = []
            for name in self._groups:
                for joint in self._groups[name].joint_names:
                    if joint not in joint_names:
                        joint_names.append(joint)
            return joint_names
        if group not in self._groups:
            return []
        return self._groups[group].joint_names

    def get_group_names(self):
        return list(self._groups.keys())

    # -------------------------------------------------------------------------
    # Commands / API
    # -------------------------------------------------------------------------

    def set_state(self, state):
        if state in self.STATE:
            self._gripper_state = state
            return True
        return False

    def get_group(self, name):
        if name not in self._groups:
            return None
        return self._groups[name]

    def set_command(self, effort):
        if self._control_joint is None:
            return
        set_point = effort
        if set_point < -self.control_joint_limits['effort']:
            set_point = -self.control_joint_limits['effort']
        elif set_point > self.control_joint_limits['effort']:
            set_point = self.control_joint_limits['effort']

        msg_ctrl = Float64()
        msg_ctrl.data = float(set_point)
        self._pubControllers[self._control_joint].publish(msg_ctrl)

        if self._mimic_joint is not None:
            msg_mimic = Float64()
            msg_mimic.data = float(self._gripper_config['mimic_joint_gain']) * float(set_point)
            self._pubControllers[self._mimic_joint].publish(msg_mimic)

    def set_controller_command(self, joint, effort):
        if joint not in self._pubControllers:
            raise RuntimeError('Invalid joint tag')
        msg = Float64()
        msg.data = float(effort)
        self._pubControllers[joint].publish(msg)
