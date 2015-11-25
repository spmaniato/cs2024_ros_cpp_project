#! /usr/bin/env python

import rospy
import actionlib
import cpp_controller_msgs.msg

def waypoint_client():
    pass


if __name__ == '__main__':
    try:
        rospy.init_node("waypoint_client_py")
        result = waypoint_client()
        print "Result:", result
    except rospy.ROSInterruptException as e:
        print "Client interrupted before completion!", str(e)
