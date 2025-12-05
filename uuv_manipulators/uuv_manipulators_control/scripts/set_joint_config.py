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

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


def main():
    # Create argument parser
    parser = argparse.ArgumentParser(
        description='uuv_manipulators_control::set_joint_config'
    )
    parser.add_argument('--namespace', metavar='NS', type=str)
    parser.add_argument('--arm', metavar='NS', type=str)
    parser.add_argument('--config', metavar='CONFIG', type=str)

    # Filter out the garbage (ROS remapping style args etc.)
    arguments = [a for a in sys.argv if ':=' not in a and '.py' not in a]

    args = parser.parse_args(arguments)

    # Manipulator's namespace
    assert args.namespace is not None, "Manipulator's namespace not given"
    namespace = args.namespace
    if not namespace.startswith('/'):
        namespace = '/' + namespace
    if not namespace.endswith('/'):
        namespace = namespace + '/'

    # Desired configuration name
    assert args.config is not None, "Manipulator's desired configuration not given"
    config = args.config

    # Manipulator name
    assert args.arm is not None, 'Manipulator name must be given'
    arm_name = args.arm

    rclpy.init()

    node = Node('set_joint_config')

    try:
        # Parameter name equivalent to the old ROS1 hierarchical namespace
        # e.g. /ns/arms/<arm_name>/default_configs/<config>
        param_name = f'{namespace}arms/{arm_name}/default_configs/{config}'

        # Declare the parameter so overrides from YAML/CLI can populate it
        node.declare_parameter(param_name, None)
        joint_pos = node.get_parameter(param_name).value

        assert joint_pos is not None, (
            f'Desired configuration not in the parameter namespace, config={config}, '
            f'param_name={param_name}'
        )

        print(f'Set joint configuration = {joint_pos}')
        print('Output topics:')

        pub = {}
        for joint in joint_pos:
            topic_name = f'{namespace}{arm_name}/{joint}/position_controller/command'
            print(f'     {joint} = {topic_name}')
            pub[joint] = node.create_publisher(Float64, topic_name, 1)

        start_time = time.perf_counter()
        duration = 5.0
        rate_hz = 100.0
        period = 1.0 / rate_hz

        node.get_logger().info(
            f'Publishing command message for {duration} seconds'
        )

        while (time.perf_counter() - start_time) <= duration and rclpy.ok():
            # No subscriptions, but spin_once just in case
            rclpy.spin_once(node, timeout_sec=0.0)

            for joint, pos in joint_pos.items():
                pos_msg = Float64()
                pos_msg.data = pos
                pub[joint].publish(pos_msg)

            time.sleep(period)

        node.get_logger().info('Finishing setting joint configuration')

    except Exception as e:
        print('uuv_manipulators_control::set_joint_config::Exception:', e)

    print('Leaving uuv_manipulators_control::set_joint_config')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
