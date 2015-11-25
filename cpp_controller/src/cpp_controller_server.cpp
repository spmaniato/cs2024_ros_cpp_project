#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cpp_controller_msgs/WaypointFollowingAction.h>

class WaypointFollowingAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<cpp_controller_msgs::WaypointFollowingAction> as_;
    std::string action_name_;
    cpp_controller_msgs::WaypointFollowingFeedback feedback_;

public:

    WaypointFollowingAction(std::string name) :
        as_(nh_, name, boost::bind(&WaypointFollowingAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
    }

    ~WaypointFollowingAction(void)
    {
    }

    void executeCB(const cpp_controller_msgs::WaypointFollowingGoalConstPtr &goal)
    {
        ros::Rate r(1);
        bool success = true;

        // feedback_.current_pose =

        ROS_INFO("Test!");

        if(success)
        {
            ROS_INFO("%s: Succeeded!", action_name_.c_str());
            as_.setSucceeded();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_following");

    WaypointFollowingAction waypoint_following(ros::this_node::getName());
    ros::spin();

    return 0;
}
