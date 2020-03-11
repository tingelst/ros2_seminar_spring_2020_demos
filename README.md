# ros2_seminar_spring_2020_demos
Demos for the spring seminars on ROS2 in the Robotics and Automation group at MTP, NTNU.

The following tutorial is based on the tutorials found at https://index.ros.org/doc/ros2/Tutorials/.

## Building a simple robot controller

### Creating a workspace

#### Source ROS2 environment.
The first step is to source the underlying ROS2 installation:
```bash 
source /opt/ros/eloquent/setup.bash
```

#### Create a workspace

Best practice is to create a new directory for every new workspace.

```bash
mkdir dev_ws
mkdir dev_ws/src
cd dev_ws/src
```

#### Clone demo repo
Ensure you’re still in the dev_ws/src directory before you clone.

```bash
git clone https://github.com/tingelst/ros2_seminar_spring_2020_demos.git
```

#### Build the workspace with colcon
From the root of your workspace (~/dev_ws), you can now build your packages using the command:

```bash
colcon build --symlink-install
```

#### Source the overlay

Before sourcing the overlay, it is very important that you open a new terminal, separate from the one where you built the workspace. Sourcing an overlay in the same terminal where you built, or likewise building where an overlay is sourced, may create complex issues.

In the new terminal, source your main ROS 2 environment as the “underlay”, so you can build the overlay “on top of” it:
```bash
source /opt/ros/eloquent/setup.bash
```
Go into the root of your workspace:
```bash
cd dev_ws
```
In the root, source your overlay:
```bash
source install/local_setup.bash
```


#### Summary
In this tutorial, you sourced your main ROS 2 distro install as your underlay, and created an overlay by cloning and building packages in a new workspace. The overlay gets prepended to the path, and takes precedence over the underlay, as you saw with your modified turtlesim.

Using overlays is recommended for working on a small number of packages, so you don’t have to put everything in the same workspace and rebuild a huge workspace on every iteration.

### Create a simple robot controller package

#### Create a package

First, source your ROS 2 installation.

Let’s use the workspace you created in the previous tutorial, `dev_ws`, for your new package.

Make sure you are in the src folder before running the package creation command.

```bash
cd ~/dev_ws/src
```

Create a new Python package with the name `simple_controller` and the node (executable) `simple_controller_node`:
```bash
ros2 pkg create --build-type ament_python --node-name simple_controller_node simple_controller
```

You will now have a new folder within your workspace’s src directory called `simple_controller`.

After running the command, your terminal will return the message:
```bash
going to create a new package
package name: simple_controller
destination directory: /home/lars/dev_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['<name> <email>']
licenses: ['TODO: License declaration']
build type: ament_python
dependencies: []
node_name: simple_controller_node
creating folder ./simple_controller
creating ./simple_controller/package.xml
creating source folder
creating folder ./simple_controller/simple_controller
creating ./simple_controller/setup.py
creating ./simple_controller/setup.cfg
creating folder ./simple_controller/resource
creating ./simple_controller/resource/simple_controller
creating ./simple_controller/simple_controller/__init__.py
creating folder ./simple_controller/test
creating ./simple_controller/test/test_copyright.py
creating ./simple_controller/test/test_flake8.py
creating ./simple_controller/test/test_pep257.py
creating ./simple_controller/simple_controller/simple_controller_node.py
```

#### Build your package

Putting packages in a workspace is especially valuable because you can build many packages at once by running colcon build in the workspace root. Otherwise, you would have to build each package individually.

Return to the root of your workspace:
```bash 
cd ~/dev_ws
```
Now you can build your packages:
```bash
colcon build --symlink-install
```

To build only the `simple_controller` package next time, you can run:
```bash
colcon build --packages-select simple_controller
```

#### Source the setup file
To use your new package and executable, first open a new terminal and source your main ROS 2 installation.

Then, from inside the dev_ws directory, run the following command to source your workspace:

```bash
source install/setup.bash
```

Now that your workspace has been added to your path, you will be able to use your new package’s executables.

#### Use the package

To run the executable you created using the --node-name argument during package creation, enter the command:
```
ros2 run simple_controller simple_controller_node
```

#### Summary

You’ve created the `simple_controller` ROS2 package to organize your code and make it easy to use for others.


### Writing a simple robot controller

In this tutorial, you will create a node that receives the state of the robot over a topic, and sends a command back to the robot driver on another topic.

Navigate into `~/dev_ws/src/simple_controller/simple_controller`:
```bash 
cd ~/dev_ws/src/simple_controller/simple_controller
```

Now there will be a new file named `simple_controller_node.py` adjacent to `__init__.py`.

Open the file using your preferred text editor, remove the code that is already there, and copy in the following:

```python
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
```

##### Examine the code

The first lines of code import the relevant modules from the Python installation:
```python
import numpy as np
import time
from copy import copy
```

Then, we import rclpy so its Node class can be used.
```python 
import rclpy
from rclpy.node import Node
```

The next statement imports the message types `JointState` and `PositionCommand` that the node uses to subscribe and publish on topics.

```python
import numpy as np
import time
from copy import copy
```

Next, the `SimpleControllerNode` class is created, which inherits from (or is a subclass of) Node.
```python
class SimpleControllerNode(Node):
```

Following is the definition of the class’s constructor. super().__init__ calls the Node class’s constructor and gives it your node name, in this case `'simple_robot_controller'`.

`create_publisher` declares that the node publishes messages of type `PositionCommand` (imported from the `control_interfaces.msg` module), over a topic named `command`, and that the “queue size” is 10. Queue size is a required QoS (quality of service) setting that limits the amount of queued messages if a subscriber is not receiving them fast enough.

```python
self._pub = self.create_publisher(PositionCommand, 'command', 10)
```

`create_subscriber` declares that the node subscribes to messages of type `JointState` on the topic `joint_states`. Each time we receive a new message on this topic the callback function `_callback` is called.

```python
self._sub = self.create_subscription(
    JointState, 'joint_states', self._callback, 10)
```

The callback function stores the time we received the first message, as well as the initial position of the robot. Then we compute a correction to this initial position as a sine wave, and then publishes the message.

```python
def _callback(self, msg):
    if self._t0 is None:
        self._t0 = time.time()
        self._initial_position = copy(msg.position)

    command_msg = PositionCommand()
    command_msg.command = copy(msg.position)

    correction = 0.1 * (np.sin(3.0 * (time.time() - self._t0)))
    command_msg.command[0] = self._initial_position[0] + correction

    self._pub.publish(command_msg)
```

Lastly, the main function is defined.
```python
def main(args=None):
    rclpy.init(args=args)

    simple_controller_node = SimpleControllerNode()
    rclpy.spin(simple_controller_node)
    
    simple_controller_node.destroy_node()
    rclpy.shutdown()
```

First the rclpy library is initialized, then the node is created, and then it “spins” the node so its callbacks are called.


### Launch the whole system

We are now ready to bringup all the required components of our system: 
1. the robot visualization
2. the robot driver
3. the robot controller

#### Visualization

The primary 3D visualization tool in ROS and ROS2 is called RViz. To bringup the 3D visualization of the robot in RViz open up a new terminal and run the following:
```bash
source ~/dev_ws/install/setup.bash
ros2 launch kuka_kr6_support test_kr6r900sixx.launch.py
```

#### Robot driver

To run the robot driver open up a new terminal and run the following:
```bash
source ~/dev_ws/install/setup.bash
ros2 run simple_robot_driver simple_robot_driver_node
```
This launches a node that subscribes to a position command message, executes the command, and publishes the current state of the robot.

#### Robot controller

We are now ready to launch the robot controller which subscribes to the current state of the robot and in each cycle publishes a new command. Open up a new terminal and run the following:
```bash
source ~/dev_ws/install/setup.bash
ros2 run simple_controller simple_controller_node 
```

If we are lucky, the robot should now move from side to side!