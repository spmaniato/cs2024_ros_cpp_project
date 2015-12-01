# A simple ROS Action server in C++

Lightweight waypoint following for the [TurtleSim](http://wiki.ros.org/turtlesim) 
in [ROS](http://www.ros.org/) using [actionlib](http://wiki.ros.org/actionlib).

![img](https://dl.dropboxusercontent.com/u/43993203/waypoint_following.png)

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

The server and client comminicate via the ROS [`actionlib`](http://wiki.ros.org/actionlib) interface.
The figure below, which is linked from http://wiki.ros.org/actionlib, illustrates this interface.

![img](http://wiki.ros.org/actionlib?action=AttachFile&do=get&target=client_server_interaction.png)

### Maintainers:
- Spyros Maniatopoulos ([@spmaniato](https://github.com/spmaniato), sm2296@cornell.edu)

### License

[BSD-3](http://opensource.org/licenses/BSD-3-Clause)

## How-to

### Installation

... 

### Running the Server and Client

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

