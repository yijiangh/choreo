#include <ros/ros.h>

#include <framefab_msgs/PathPostprocessing.h>
#include <framefab_path_post_processor/path_post_processor.h>

bool processPath(framefab_msgs::PathPostProcessingRequest& req,
             framefab_msgs::PathPostProcessingResponse& res)
{
  framefab_path_post_processing::PathPostProcessor path_pprocessor;
  path_pprocessor.setPathInputParams(req.params);

  switch (req.action)
  {
    case framefab_msgs::PathPostProcessing::Request::PROCESS_PATH_AND_MARKER:
    {
      if(!path_pprocessor.createCandidatePoses())
      {
        ROS_ERROR("Could not post process input path!");
        res.succeeded = false;
        return false;
      }
      res.succeeded = true;
      break;
    }

    default:
    {
      ROS_ERROR_STREAM("Unrecognized path post processing request");
      res.succeeded = false;
      return false;
    }
  }

  res.process = path_pprocessor.getCandidatePoses();
  return true;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "path_post_processor");
  ros::NodeHandle nh;

  ros::ServiceServer path_post_process_server =
      nh.advertiseService<framefab_msgs::PathPostprocessingRequest, framefab_msgs::PathPostprocessingResponse>(
          "path_post_process", boost::bind(processPath, _1, _2));

  ROS_INFO("%s ready to service requests.", path_generator_server.getService().c_str());
  ros::spin();

  return 0;
}