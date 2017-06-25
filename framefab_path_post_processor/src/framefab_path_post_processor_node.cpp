// srv
#include <framefab_msgs/PathPlanning.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "path_post_processor");
  ros::NodeHandle nh;

  ros::ServiceServer path_post_processor =
      nh.advertiseService<framefab_msgs::PathPlanningRequest, godel_msgs::PathPlanningResponse>(
          "process_path_generator", boost::bind(pathGen, _1, _2, boundary_offset_client));
  ROS_INFO("%s ready to service requests.", path_generator.getService().c_str());
  ros::spin();

  return 0;
}