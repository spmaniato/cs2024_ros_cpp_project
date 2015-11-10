#pragma once
#include <ros/ros.h>
#include "turtlesim/Pose.h"

class CppControllerNode {

public:
    CppControllerNode();
    virtual void init(ros::NodeHandle&);
protected:
    turtlesim::Pose target_position_;
    // callbacks
    void poseCallback(const turtlesim::Pose&);
    geometry_msgs::Twist calculateControlInputs(turtlesim::Pose);
    // subscribers
    ros::Subscriber pose_sub_;
    // publishers
    ros::Publisher controller_pub_;
};
