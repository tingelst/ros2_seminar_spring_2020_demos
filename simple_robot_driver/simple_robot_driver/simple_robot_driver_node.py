# Copyright 2019 Norwegian University of Science and Technology
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

import numpy as np
from copy import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_interfaces.msg import PositionCommand


class SimpleRobotDriverNode(Node):
    def __init__(self):
        super().__init__('simple_robot_driver')
        self._pub = self.create_publisher(JointState, 'joint_states', 10)
        self._sub = self.create_subscription(
            PositionCommand, 'command', self._command_callback, 10)
        self._timer = self.create_timer(0.01, self._timer_callback)

        self._position = [0.0, -1.5708, 1.5708, 0.0, 1.5708, 0.0]
        self._velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def _timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_a1', 'joint_a2', 'joint_a3',
                    'joint_a4', 'joint_a5', 'joint_a6']
        msg.position = copy(self._position)
        msg.velocity = copy(self._velocity)
        self._pub.publish(msg)

    def _command_callback(self, msg):
        self._position = copy(msg.command)


def main(args=None):
    rclpy.init(args=args)

    simple_robot_driver_node = SimpleRobotDriverNode()
    rclpy.spin(simple_robot_driver_node)

    simple_robot_driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
