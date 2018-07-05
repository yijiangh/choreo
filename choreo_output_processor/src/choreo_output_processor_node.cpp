//
// Created by yijiangh on 9/10/17.
//

#include <ros/ros.h>
#include <ros/console.h>

// srv
#include <choreo_msgs/OutputProcessing.h>

#include <choreo_output_processor/choreo_output_processor.h>

const static std::string OUTPUT_PROCESSING_SERVICE = "output_processing";

bool outputProcessingCallback(choreo_msgs::OutputProcessingRequest& req,
                              choreo_msgs::OutputProcessingResponse& res)
{
  choreo_output_processor::OutputProcessor op;
  op.setSaveFilePath(req.file_path);

  if(op.outputJson(req.plans))
  {
    res.succeeded = true;
    return true;
  }
  else
  {
    res.succeeded = false;
    return false;
  }
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "choreo_output_processor");
  ros::NodeHandle nh;

  ros::ServiceServer output_processing_server =
      nh.advertiseService<choreo_msgs::OutputProcessingRequest, choreo_msgs::OutputProcessingResponse>(
          OUTPUT_PROCESSING_SERVICE, boost::bind(outputProcessingCallback, _1, _2));

  ROS_INFO("%s server ready to service requests.", output_processing_server.getService().c_str());
  ros::spin();

  return 0;
}