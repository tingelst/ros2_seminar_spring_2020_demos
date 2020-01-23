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
import time
from copy import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_interfaces.msg import PositionCommand


class SimpleControllerNode(Node):
    def __init__(self):
        super().__init__('simple_robot_controller')
        self._pub = self.create_publisher(PositionCommand, 'command', 10)
        self._sub = self.create_subscription(
            JointState, 'joint_states', self._callback, 10)

        self._t0 = None
        self._initial_position = None

    def _callback(self, msg):
        if self._t0 is None:
            self._t0 = time.time()
            self._initial_position = copy(msg.position)

        command_msg = PositionCommand()
        command_msg.command = copy(msg.position)

        correction = 0.1 * (np.sin(3.0 * (time.time() - self._t0)))
        command_msg.command[0] = self._initial_position[0] + correction

        self._pub.publish(command_msg)


def main(args=None):
    rclpy.init(args=args)

    simple_controller_node = SimpleControllerNode()
    rclpy.spin(simple_controller_node)
    
    simple_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
