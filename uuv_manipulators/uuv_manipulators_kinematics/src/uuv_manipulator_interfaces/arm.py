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

from copy import deepcopy
from typing import Callable, Dict, List, Optional

import numpy as np
import PyKDL

import rclpy
from rclpy.node import Node

from uuv_manipulators_msgs.msg import EndPointState
from uuv_kinematics_utils import EndEffectorState
from uuv_manipulator_interfaces.kin_chain import KinChainInterface
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf_quaternion.transformations import quaternion_from_euler, euler_from_quaternion


class ArmInterface(KinChainInterface):
    """Interface class to the arm."""

    STATE = ['moving', 'disabled']

    def __init__(
        self,
        node: Node,
        namespace: Optional[str] = None,
        arm_name: Optional[str] = None,
        compute_fk_for_all: bool = False
    ):
        """
        :param node: rclpy Node that owns parameters/subscriptions
        :param namespace: manipulator namespace (optional; derived from node namespace if None)
        :param arm_name: arm name (optional; derived from node namespace if None)
        :param compute_fk_for_all: if True, compute FK for all links
        """
        self._node = node

        # Derive namespace and arm_name from node namespace if not given
        if None in [namespace, arm_name]:
            ns_tokens = [item for item in node.get_namespace().split('/') if len(item) > 0]
            if len(ns_tokens) != 2:
                raise RuntimeError(
                    'The controller must be called inside the namespace '
                    '<manipulator_namespace>/<arm_name>'
                )
            namespace = ns_tokens[0]
            arm_name = ns_tokens[1]

        # Use dot-separated params for ROS 2
        param_prefix = f'arms.{arm_name}'

        # Callbacks called in topic handlers
        self._callbacks: Dict[str, List[Callable]] = {}

        # Published topics (empty for now, kept for compatibility)
        self._pubTopics: Dict[str, object] = {}
        # Subscribed topics
        self._subTopics: Dict[str, object] = {}

        # Initializing the end-effector state
        self._endeffector_state = EndEffectorState()

        # Helper methods to access parameters in ROS 2
        def _get_param(name: str, default=None):
            # Declare parameter if not declared already; if a value is
            # provided from YAML/CLI/launch, it will override the default.
            if not self._node.has_parameter(name):
                self._node.declare_parameter(name, default)
            return self._node.get_parameter(name).value

        # Retrieve default configurations (may be None)
        default_configs_param = f'{param_prefix}.default_configs'
        self._default_configs = _get_param(default_configs_param, None)

        self._man_index = 0.0

        # Base & tip link parameters (from launch)
        base_link_param = f'{param_prefix}.base_link'
        tip_link_param = f'{param_prefix}.tip_link'

        # Retrieve the names of the base and tip links
        base_link = _get_param(base_link_param, None)
        tip_link = _get_param(tip_link_param, None)

        if base_link is None:
            raise RuntimeError(
                f'Base link name not available in the namespace {namespace} (param {base_link_param})'
            )
        if tip_link is None:
            raise RuntimeError(
                f'Tip link name not available in the namespace {namespace} (param {tip_link_param})'
            )

        # Initialize kinematics chain interface
        KinChainInterface.__init__(
            self,
            node=self._node,
            name=arm_name,
            base=base_link,
            ee_link=tip_link,
            namespace=namespace,
            arm_name=arm_name,
            compute_fk_for_all=compute_fk_for_all
        )

        self.set_joint_names(self._joint_names)

        # In ROS 2, QoS is used instead of tcp_nodelay, queue_size, etc.
        joint_states_topic = f'/{namespace}/{arm_name}/joint_states'
        self._subTopics['joint_states'] = self._node.create_subscription(
            JointState,
            joint_states_topic,
            self._on_joint_states,
            1  # queue size
        )

    def __del__(self):
        """Class destructor."""
        # Properly destroy subscriptions and publishers in ROS 2
        for topic in list(self._subTopics.keys()):
            sub = self._subTopics[topic]
            try:
                self._node.destroy_subscription(sub)
            except Exception:
                pass
        for topic in list(self._pubTopics.keys()):
            pub = self._pubTopics[topic]
            try:
                self._node.destroy_publisher(pub)
            except Exception:
                pass

    @property
    def endeffector_pose(self):
        return dict(position=self._endeffector_state.position,
                    orientation=self._endeffector_state.orientation)

    @property
    def endeffector_twist(self):
        return dict(linear=self._endeffector_state.linear_velocity,
                    angular=self._endeffector_state.angular_velocity)

    @property
    def endeffector_wrench(self):
        return dict(force=self._endeffector_state.force,
                    torque=self._endeffector_state.torque)
    @property
    def home(self):
        """
        Returns the joint configuration of the home state.

        If no default "home" configuration is provided via parameters,
        fall back to a zero configuration for all joints.
        """
        if self._default_configs and 'home' in self._default_configs:
            return self._default_configs['home']

        # Fallback: zero position for all joints
        if hasattr(self, '_joint_names') and self._joint_names:
            return {j: 0.0 for j in self._joint_names}

        return {}

    @property
    def default_configs(self):
        return self._default_configs

    @property
    def man_index(self):
        """
        Returns the manipulability index
        """
        return self._man_index

    def _on_joint_states(self, msg: JointState):
        # Store the joint states
        self.update_joint_states(msg)

        # Update manipulability index
        self.update_man_index()
        # Update end-effector state
        self.update_endeffector_state()

        if 'joint_states' in self._callbacks:
            if len(self._callbacks['joint_states']):
                for fcn in self._callbacks['joint_states']:
                    fcn()

    def update_man_index(self):
        # Retrieve current jacobian matrix
        J = self.jacobian()
        det = np.linalg.det(np.dot(J, J.transpose()))
        self._man_index = 0.0
        if det > 0:
            self._man_index = np.sqrt(det)

    def update_endeffector_state(self):
        # Read joint states from the kinematics interface
        q = self.joint_angles
        qd = self.joint_velocities
        eff = self.joint_efforts

        # Compute the pose of all frames, if requested
        if self._compute_fk_for_all:
            for idx in range(self.n_links):
                self._frames[idx] = self.forward_position_kinematics(q, segment_idx=idx)

        # End effector pose
        pose = self.forward_position_kinematics(q)
        vel = self.forward_velocity_kinematics(q, qd)

        # In Python 3, eff.values() is a view; convert to list
        wrench = np.dot(self.jacobian(q), np.array(list(eff.values())))

        # Store everything in the end point state message
        self._endeffector_state.position = pose[0:3]
        self._endeffector_state.orientation = pose[3::]

        self._endeffector_state.linear_velocity = np.array([
            vel.vel.x(),
            vel.vel.y(),
            vel.vel.z()
        ])
        self._endeffector_state.angular_velocity = np.array([
            vel.rot.x(),
            vel.rot.y(),
            vel.rot.z()
        ])
        self._endeffector_state.force = np.array([
            wrench[0, 0],
            wrench[0, 1],
            wrench[0, 2]
        ])
        self._endeffector_state.torque = np.array([
            wrench[0, 3],
            wrench[0, 4],
            wrench[0, 5]
        ])

    def set_joint_names(self, jnt_names):
        self._joint_names = jnt_names

        # Adapting the joint names to the namespace
        if self._default_configs is not None:
            new_configs = {}
            for config in self._default_configs:
                new_configs[config] = {}
                for joint in self._default_configs[config]:
                    for j in self._joint_names:
                        if joint in j:
                            new_configs[config][j] = self._default_configs[config][joint]
            self._default_configs = new_configs

    def get_ee_pose_as_msg(self):
        return self._endeffector_state.to_msg()

    def get_ee_pose_as_frame(self):
        return self._endeffector_state.to_frame()

    def get_ee_vel_as_kdl_twist(self):
        return self._endeffector_state.to_kdl_twist()

    def get_config_in_ee_frame(self, config='home'):
        assert self._default_configs is not None and config in self._default_configs, \
            'Invalid default configuration'

        j_pos = self._default_configs[config]
        pose = self.forward_position_kinematics(j_pos)

        quaternion = (pose[3], pose[4], pose[5], pose[6])

        euler = euler_from_quaternion(quaternion)
        frame = PyKDL.Frame(
            PyKDL.Rotation.RPY(euler[0], euler[1], euler[2]),
            PyKDL.Vector(pose[0], pose[1], pose[2])
        )
        return frame

    def get_config(self, config='home'):
        assert self._default_configs is not None and config in self._default_configs, \
            'Invalid default configuration'
        return self._default_configs[config]

    def add_callback(self, topic_name: str, function_handle: Callable):
        if topic_name not in self._subTopics:
            print('ArmInterface - Invalid topic name')
            return

        if topic_name not in self._callbacks:
            self._callbacks[topic_name] = []

        self._callbacks[topic_name].append(function_handle)
