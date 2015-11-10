#include <cpp_controller/cpp_controller_node.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cpp_controller");

  // ros::NodeHandle nh;
  // vigir_footstep_planning::FootstepPlannerNode footstep_planner_node;
  // footstep_planner_node.init(nh);

  ros::spin();

  return 0;
}
