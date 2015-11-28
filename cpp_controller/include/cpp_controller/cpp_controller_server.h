#pragma once
#include <iostream>

#include <ros/ros.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <cpp_controller_msgs/WaypointFollowingAction.h>

using std::string;
using cpp_controller_msgs::WaypointFollowingAction;
using cpp_controller_msgs::WaypointFollowingFeedback;
using cpp_controller_msgs::WaypointFollowingGoalConstPtr;

class WaypointFollowingActionServer
{

public:
    WaypointFollowingActionServer(string name);
    ~WaypointFollowingActionServer(void);

    void executeCB(const WaypointFollowingGoalConstPtr &goal);

protected:

    // Action server -related member variables
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<WaypointFollowingAction> as_;
    string action_name_;
    WaypointFollowingFeedback feedback_;

    // Member variable for keeping track of the robot's pose
    geometry_msgs::Pose2D current_pose_;

    float threshold_;
    float epsilon_;

    // subscribers
    ros::Subscriber pose_sub_;
    // publishers
    ros::Publisher controller_pub_;

    void poseCallback(const turtlesim::Pose&);
    bool closeEnough(geometry_msgs::Pose2D);
    geometry_msgs::Twist calculateControlInputs(geometry_msgs::Pose2D);
};
