#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

#include <sstream>


// void CppControllerNodeInit(ros::NodeHandle &nh) {
//
//     // Subscribe to these topics
//
//     // Publish to these topics
//
// }

void poseCallback(const turtlesim::Pose pose_msg) {
    ROS_INFO("(%f, %f)", pose_msg.x, pose_msg.y);
    // return pose_msg
}

geometry_msgs::Twist calculateControlInputs(turtlesim::Pose pose_msg) {
    geometry_msgs::Twist controls_msg;
    controls_msg.linear.x = 2.0;
    controls_msg.angular.z = 2.0;
    return controls_msg;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "cpp_controller");

    ros::NodeHandle nh;

    ros::Publisher controller_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    ros::Subscriber pose_sub = nh.subscribe("turtle1/pose", 1000, poseCallback);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

    turtlesim::Pose pose = turtlesim::Pose();

    geometry_msgs::Twist controls_msg = calculateControlInputs(pose);

    // ROS_INFO("%s", controls_msg); //TODO: How to do this in cpp?

    controller_pub.publish(controls_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
