#! /usr/bin/env python

import rospy
import actionlib

from cpp_controller_msgs.msg import *
from geometry_msgs.msg import Pose2D

def waypoint_client(waypoints = []):

    # Create the client, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient("waypoint_following",
                                          WaypointFollowingAction)
    # Wait until the action server has started up.
    client.wait_for_server()

    # Create a goal to be sent to the action server.
    action_goal = WaypointFollowingGoal()

    # Fill out the request part of the message.
    action_goal.waypoints = waypoints

    # Send the goal to the   action server.
    client.send_goal(action_goal)

    # Wait for the server to finish performing the action.
    client.wait_for_result()

    return client.get_state()

if __name__ == '__main__':

    waypoints = list()
    waypoints.append(Pose2D(x = 9.0, y = 1.0))
    waypoints.append(Pose2D(x = 9.0, y = 9.0))
    waypoints.append(Pose2D(x = 1.0, y = 9.0))
    waypoints.append(Pose2D(x = 1.0, y = 1.0))

    for wp in waypoints:
        print wp

    try:
        rospy.init_node("waypoint_client_py")
        result = waypoint_client(waypoints)
        print "Result:", result, ("SUCCEEDED = 3")
    except rospy.ROSInterruptException as e:
        print "Client interrupted before completion!", str(e)
