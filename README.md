# ros2_seminar_spring_2020_demos
Demos for the spring seminar on ROS2 in the Robotics and Automation group at MTP, NTNU.

The following tutorial is based on the tutorials found at https://index.ros.org/doc/ros2/Tutorials/.

## Building a simple velocity controller

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

#### Resolve dependencies

From the root of your workspace (~/dev_ws), you need to resolve package dependencies.

```bash
sudo rosdep install -i --from-path src --rosdistro eloquent -y
```

If you already have all your dependencies, the console will return:
```bash
#All required rosdeps installed successfully
```

#### Build the workspace with colcon
From the root of your workspace (~/dev_ws), you can now build your packages using the command:

```bash
colcon build
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
colcon build
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


