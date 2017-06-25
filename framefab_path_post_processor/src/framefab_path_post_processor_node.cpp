
// srv
#include <framefab_msgs/PathPlanning.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "process_path_generator");
  ros::NodeHandle nh;

  // waiting for service
  while (!ros::service::waitForService(OFFSET_POLYGON_SERVICE, ros::Duration(10.0f)))
  {
    ROS_WARN_STREAM("Connecting to service '" << OFFSET_POLYGON_SERVICE << "'");
  }

  ros::ServiceClientPtr boundary_offset_client(
      new ros::ServiceClient(nh.serviceClient<godel_msgs::OffsetBoundary>(OFFSET_POLYGON_SERVICE)));

  ros::ServiceServer path_generator =
      nh.advertiseService<godel_msgs::PathPlanningRequest, godel_msgs::PathPlanningResponse>(
          "process_path_generator", boost::bind(pathGen, _1, _2, boundary_offset_client));
  ROS_INFO("%s ready to service requests.", path_generator.getService().c_str());
  ros::spin();

  return 0;
}
