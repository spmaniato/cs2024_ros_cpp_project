#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

// #include <cpp_controller_msgs/WaypointFollowingAction.h>
#include <cpp_controller/cpp_controller_server.h>

using std::string;

WaypointFollowingActionServer::WaypointFollowingActionServer(string name) :
        as_(nh_, name, boost::bind(&WaypointFollowingActionServer::executeCB, this, _1), false),
        action_name_(name)
{
    as_.start();

    // Subscribe to these topics
    pose_sub_ = nh_.subscribe("turtle1/pose", 1, &WaypointFollowingActionServer::poseCallback, this);

    // Publish to these topics
    controller_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

    // Controller parameters
    threshold_ = 0.5;
    epsilon_ = 3.0;
}

WaypointFollowingActionServer::~WaypointFollowingActionServer(void)
{
}

void WaypointFollowingActionServer::executeCB(const cpp_controller_msgs::WaypointFollowingGoalConstPtr &goal)
{
    ros::Rate r(1);
    bool success = true; // Start optimistically
    feedback_.next_waypoint = 0;

    for (auto wp : goal->waypoints)
    {
        ROS_INFO("Navigating to waypoint: (x = %f, y = %f)", wp.x, wp.y);
        as_.publishFeedback(feedback_);

        while (!closeEnough(wp))
        {
            ROS_INFO_THROTTLE(1, "Robot's pose: (x = %f, y = %f)", current_pose_.x, current_pose_.y);
            geometry_msgs::Twist controls_msg = calculateControlInputs(wp);
            ROS_INFO_THROTTLE(1, "Calculated control inputs: (u = %f, w = %f)", controls_msg.linear.x, controls_msg.angular.z);
            controller_pub_.publish(controls_msg);
        }

        feedback_.next_waypoint++;
    }

    if(success)
    {
        ROS_INFO("%s action succeeded!", action_name_.c_str());
        as_.setSucceeded();
    }
}

bool WaypointFollowingActionServer::closeEnough(geometry_msgs::Pose2D target)
{
    float norm = std::sqrt(std::pow((target.x - current_pose_.x), 2) + std::pow((target.y - current_pose_.y), 2));

    return (norm < threshold_);
}

void WaypointFollowingActionServer::poseCallback(const turtlesim::Pose &pose_msg) {

    current_pose_.x = pose_msg.x;
    current_pose_.y = pose_msg.y;
    current_pose_.theta = pose_msg.theta;
}

geometry_msgs::Twist WaypointFollowingActionServer::calculateControlInputs(geometry_msgs::Pose2D target_pose)
{
    float vx = target_pose.x - current_pose_.x;
    float vy = target_pose.y - current_pose_.y;
    float theta = current_pose_.theta;

    float u = vx*std::cos(theta) + vy*std::sin(theta);
    float w = (1.0/epsilon_)*(-vx*std::sin(theta) + vy*std::cos(theta));

    //TODO: Add control input saturation constraints. It turns too fast.

    geometry_msgs::Twist controls_msg;
    controls_msg.linear.x = u;
    controls_msg.angular.z = w;

    return controls_msg;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_following");

    WaypointFollowingActionServer waypoint_following(ros::this_node::getName());
    ros::spin();

    return 0;
}
