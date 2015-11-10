#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

#include <cpp_controller/cpp_controller_node.h>

#include <math.h>

CppControllerNode::CppControllerNode() {

    target_position_.x = 9.5;
    target_position_.y = 5.0;
}

void CppControllerNode::init(ros::NodeHandle &nh) {

    // Subscribe to these topics
    pose_sub_ = nh.subscribe("turtle1/pose", 1, &CppControllerNode::poseCallback, this);

    // Publish to these topics
    controller_pub_ = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
}

void CppControllerNode::poseCallback(const turtlesim::Pose &pose_msg) {

    ROS_INFO("(x=%f, y=%f)", pose_msg.x, pose_msg.y);

    geometry_msgs::Twist controls_msg = this->calculateControlInputs(pose_msg);
    controller_pub_.publish(controls_msg);
}

geometry_msgs::Twist CppControllerNode::calculateControlInputs(turtlesim::Pose pose_msg) {

    float epsilon = 2.0;
    float vx = target_position_.x - pose_msg.x;
    float vy = target_position_.y - pose_msg.y;
    float theta = pose_msg.theta;

    float u = vx*std::cos(theta) + vy*std::sin(theta);
    float w = (1.0/epsilon)*(-vx*std::sin(theta) + vy*std::cos(theta));

    //TODO: Add control input saturation constraints. It turns too fast.

    geometry_msgs::Twist controls_msg;
    controls_msg.linear.x = u;
    controls_msg.angular.z = w;

    ROS_INFO("(u=%f, w=%f)", u, w);
    return controls_msg;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "cpp_controller");

    ros::NodeHandle nh;
    CppControllerNode cpp_controller_node = CppControllerNode();
    cpp_controller_node.init(nh);

    ros::Rate loop_rate(1);

    ROS_INFO("Done initializing! About to start spinning.");
    ros::spin();

    ROS_INFO("I'm spinning!");
    return 0;
}
