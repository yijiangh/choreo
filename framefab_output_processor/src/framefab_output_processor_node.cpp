//
// Created by yijiangh on 9/10/17.
//

#include <ros/ros.h>
#include <ros/console.h>

// srv
#include <framefab_msgs/OutputProcessing.h>

#include <framefab_output_processor/framefab_output_processor.h>

const static std::string OUTPUT_PROCESSING_SERVICE = "output_processing";

bool outputProcessingCallback(framefab_msgs::OutputProcessingRequest& req,
                         framefab_msgs::OutputProcessingResponse& res)
{
  res.succeeded = true;
  return true;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "framefab_output_processor");
  ros::NodeHandle nh;

  ros::ServiceServer output_processing_server =
      nh.advertiseService<framefab_msgs::OutputProcessingRequest, framefab_msgs::OutputProcessingResponse>(
          OUTPUT_PROCESSING_SERVICE, boost::bind(outputProcessingCallback, _1, _2));

  ROS_INFO("%s server ready to service requests.", output_processing_server.getService().c_str());
  ros::spin();

  return 0;
}