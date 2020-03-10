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
import socket
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_interfaces.msg import PositionCommand


class RpiJointJoggerNode(Node):
    def __init__(self):
        super().__init__('simple_robot_controller')

        self._sensor_host = '192.168.0.10'
        self._sensor_port = 50007
        self._sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sensor_socket.connect((self._sensor_host, self._sensor_port))

        self._pub = self.create_publisher(PositionCommand, 'command', 10)
        self._sub = self.create_subscription(
            JointState, 'joint_states', self._callback, 10)

        self._active_joint = 0

    def __del__(self):
        self._sensor_socket.close()

    def _callback(self, msg):

        command_msg = PositionCommand()
        command_msg.command = copy(msg.position)

        self._sensor_socket.sendall(struct.pack('I', self._active_joint + 1))
        event = self._sensor_socket.recv(1024)

        if event == b'up':
            if self._active_joint == 5:
                self._active_joint = 0
            else:
                self._active_joint += 1
        elif event == b'down':
            if self._active_joint == 0:
                self._active_joint = 5
            else:
                self._active_joint -= 1
        elif event == b'left':
            command_msg.command[self._active_joint] += 0.1
            self._pub.publish(command_msg)
        elif event == b'right':
            command_msg.command[self._active_joint] -= 0.1
            self._pub.publish(command_msg)


def main(args=None):
    rclpy.init(args=args)

    rpi_joint_jogger_node = RpiJointJoggerNode()
    rclpy.spin(rpi_joint_jogger_node)

    rpi_joint_jogger_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
