# A simple ROS Action server in C++

Lightweight waypoint following for the [TurtleSim](http://wiki.ros.org/turtlesim) 
in [ROS](http://www.ros.org/) using [actionlib](http://wiki.ros.org/actionlib).

![img](https://dl.dropboxusercontent.com/u/43993203/img/waypoint_following.png)

### Table of Contents

* [About the Project](https://github.com/spmaniato/cs2024_ros_cpp_project#about)
* [How-to Guide](https://github.com/spmaniato/cs2024_ros_cpp_project#how-to-guide)
* [Under the Hood](https://github.com/spmaniato/cs2024_ros_cpp_project#under-the-hood)

## About

This project is meant as a more interesting 
[Simple Action Server tutorial](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29), 
where you get to write a server that interacts with other subsystems, specifically, TurtleSim.
It comprises three ROS ([Robot Operating System)](http://wiki.ros.org/) packages:

* The `cpp_controller` package is the meat of the project.
It contains a simple action server, written in C++. 
Upon request, it steers the robot (the turtle in TurtleSim) between a series of 2D waypoints.
* The `cpp_controller_msgs` defines the `WaypointFollowing` action interface.
* The `py_controller_client` is there to house a simple Python action client to the C++ server.
The list of waypoints is defined by the client and sent to the server via the aforementioned action interface.

The server and client comminicate via the ROS [actionlib](http://wiki.ros.org/actionlib) interface.
The figure below, which is linked from http://wiki.ros.org/actionlib, illustrates this interface.

![img](http://wiki.ros.org/actionlib?action=AttachFile&do=get&target=client_server_interaction.png)

### Maintainers:

Spyros Maniatopoulos ([@spmaniato](https://github.com/spmaniato), sm2296@cornell.edu)

### License

[BSD-3](http://opensource.org/licenses/BSD-3-Clause) (see [`LICENSE`](https://github.com/spmaniato/cs2024_ros_cpp_project/blob/master/LICENSE) file)

## How-to Guide

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
catkin_init_workspace # Make sure you are in the src directory
```

Then, navigate to the `src` directory and clone this project:
```bash
cd ~/catkin_ws/src # Make sure you are in the src directory
git clone https://github.com/spmaniato/cs2024_ros_cpp_project.git
```

Almost there. Now source the workspace and compile the code:
```bash
cd .. # Takes you back to the catkin_ws directory
source devel/setup.bash
catkin_make # You may have to run this twice
```

Double-check that your catkin workspace is in the ROS package path:
```bash
echo $ROS_PACKAGE_PATH
```

### Running TurtleSim, the Server, and the Client

To launch the **master** node, the TurtleSim simulation node, 
and our server run the following commands in three separate terminals:
```bash
roscore
rosrun turtlesim turtlesim_node
rosrun cpp_controller cpp_controller_server
```

**Aside**: You must have sourced your new workspace in all the terminals 
that you want to run the server and client in. 
Repeat the command in the setup instructions or add it your `.bashrc`:
```bash
source ~/catkin_ws/devel/setup.bash # Assuming you created catkin_ws under ~
```

Then, you can send a request to the server by running the client (in a fourth terminal):
```bash
rosrun py_controller_client waypoint_client.py
```

Here is some sample server output:
```
[ INFO] [1448995509.625577083]: Navigating to waypoint: (x = 1.000000, y = 1.000000)
[ INFO] [1448995509.636682061]: Robot's pose: (x = 0.907952, y = 9.490376)
[ INFO] [1448995509.658199932]: Calculated control inputs: (u = 5.635375, w = 2.065123)
[ INFO] [1448995510.636689515]: Robot's pose: (x = 0.009242, y = 4.545070)
[ INFO] [1448995510.658202057]: Calculated control inputs: (u = 3.525378, w = 0.281630)
[ INFO] [1448995511.636694146]: Robot's pose: (x = 0.307181, y = 2.256353)
[ INFO] [1448995511.658201677]: Calculated control inputs: (u = 1.336730, w = 0.130230)
[ INFO] [1448995512.636698631]: Robot's pose: (x = 0.550110, y = 1.395591)
[ INFO] [1448995512.658210281]: Calculated control inputs: (u = 0.514108, w = 0.097712)
[ INFO] [1448995512.889579574]: /waypoint_following action succeeded!
```

And the corresponding client output:
```
x: 1.0
y: 1.0
Action result: 3 (SUCCEEDED = 3)
```

When you are done, `ctrl-c` all four terminals.

## Under the Hood

Given a waypoint `[x_d y_d]`, the action server determines the robot's linear, `u`, 
and angular, `ω`, velocities using the feedback linearization scheme below, 
where `ε` (epsilon) is a small positive constant parameter.

![img](https://dl.dropboxusercontent.com/u/43993203/feedback_linearization.png)

and `[x y θ]` is the pose (position and orientation) of the robot at any given time. 

The action server publishes the linear and angular velocity to the `/turtle1/cmd_vel` topic.
It listens for the robot's pose on the `/turtle1/pose` topic. See [`cpp_controller_server.cpp`](https://github.com/spmaniato/cs2024_ros_cpp_project/blob/master/cpp_controller/src/cpp_controller_server.cpp).

Once the robot is close enough to the current waypoint, the server 
switches to the next waypoint until the robot has visited them all.
Once finished, the server reports success via the action interface.
The server's node remains up and running, waiting for the next request.
