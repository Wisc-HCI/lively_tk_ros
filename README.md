# LivelyIK

*LivelyIK Front-End and Configurator for ROS2*

## Installation:

### ROS Packages
You need the following repositories in your workspace's src folder:

- [tf2_web_republisher_py](https://github.com/Wisc-HCI/tf2_web_republisher_py) `git clone https://github.com/Wisc-HCI/tf2_web_republisher_py.git`
- [rosbridge_suite (ros2 branch)](https://github.com/RobotWebTools/rosbridge_suite) `git clone -b ros2 https://github.com/RobotWebTools/rosbridge_suite.git`

For robots, we are currently testing with the Franka Emika Panda, which has a description included here:
- [moveit_resources (ros2 branch)](https://github.com/ros-planning/moveit_resources) `git clone -b ros2 https://github.com/ros-planning/moveit_resources.git`

### PIP Packages

`python3 -m pip install lively_ik_core scikit-learn numpy python-fcl urdf-parser-py`

### Other

If you don't already have it, you will also need to [Install Node.js](https://nodejs.org/en/download/).

## Running

After running `colcon build` on the workspace and sourcing, you will be able to launch the backend by executing:

`ros2 launch lively_ik lively_ik.py`

You will also need to launch the front-end application, which you can do by navigating to the gui directory of lively_ik:

`cd src/lively_ik/lively_ik_gui/`

and then

`npm start`

The application should be accessible at `http://localhost:3000`
