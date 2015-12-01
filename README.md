# A simple ROS Action server in C++

Lightweight waypoint following for the [TurtleSim](http://wiki.ros.org/turtlesim) 
in [ROS](http://www.ros.org/) using [actionlib](http://wiki.ros.org/actionlib).

![img](https://dl.dropboxusercontent.com/u/43993203/waypoint_following.png)

### Table of Contents

* [About](https://github.com/spmaniato/cs2024_ros_cpp_project#about)
* [How-to](https://github.com/spmaniato/cs2024_ros_cpp_project#how-to)
* [Under the Hood](https://github.com/spmaniato/cs2024_ros_cpp_project#under-the-hood)

## About

This project is meant as a more interesting 
[Simple Action Server tutorial](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29), 
where you get to write a server that interacts with other subsystems, specifically, TurtleSim.
It comprises three ROS ([Robot Operating System)](http://wiki.ros.org/) packages:

* The `cpp_controller` package is the meat of the project.
It contains a simple action server, written in C++, for steering a robot 
(the turtle in TurtleSim) between a series of 2D waypoints.
* The `cpp_controller_msgs` defines the `WaypointFollowing` action interface.
* The `py_controller_client` is there to house a simple Python action client to the C++ server.
The list of waypoints is defined by the client and sent to the server via the aforementioned action interface.

The server and client comminicate via the ROS [actionlib](http://wiki.ros.org/actionlib) interface.
The figure below, which is linked from http://wiki.ros.org/actionlib, illustrates this interface.

![img](http://wiki.ros.org/actionlib?action=AttachFile&do=get&target=client_server_interaction.png)

### Maintainers:
- Spyros Maniatopoulos ([@spmaniato](https://github.com/spmaniato), sm2296@cornell.edu)

### License

[BSD-3](http://opensource.org/licenses/BSD-3-Clause)

## How-to

### ROS Installation

First of all, you will need to install ROS.
My recommendation is to install either 
ROS [Indigo Igloo](http://wiki.ros.org/indigo/Installation/Ubuntu) or 
[Jade Turtle](http://wiki.ros.org/jade/Installation/Ubuntu) in 
[Ubuntu 14.04.x](http://releases.ubuntu.com/trusty/) (Trusty Tahr).
I developed and tested in Indigo and Ubuntu 14.04.3 (both native and in a virtual machine).
Follow the installation instructions very carefully and do not skip any steps.
In theory, you can also [install ROS on Mac OS X](https://github.com/mikepurvis/ros-install-osx).

### Cloning this Project

Once you have installed ROS, follow this tutorial to 
[Create a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
In a nutshell:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

Then, navigate to the `src` directory and clone this project:
```bash
cd ~/catkin_ws/src
git clone https://github.com/spmaniato/cs2024_ros_cpp_project.git
```

Almost there. Now compile the code and source the workspace:
```bash
cd ..
catkin_make
source devel/setup.bash
```

Double-check that your catkin workspace is in the ROS package path:
```bash
echo $ROS_PACKAGE_PATH
```


### Running TurtleSim, the Server, and the Client

To launch the **master** node, the TurtleSim simulation, and the server
run the following commands in three separate terminals:
```bash
roscore
rosrun turtlesim turtlesim_node
rosrun cpp_controller cpp_controller_server
```

Then, you can send a request to the server by running the client (in a fourth terminal):
```bash
rosrun py_controller_client waypoint_client.py
```

When you are done, `ctrl-c` all four terminals.

## Under the Hood

...
