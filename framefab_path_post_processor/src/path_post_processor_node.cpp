#include <ros/ros.h>
#include <ros/console.h>

#include <boost/tuple/tuple.hpp>

#include <framefab_msgs/PathPostProcessing.h>
#include <framefab_path_post_processor/process_path.h>
#include <framefab_path_post_processor/path_post_processor.h>

bool processPathCallback(framefab_msgs::PathPostProcessingRequest& req,
             framefab_msgs::PathPostProcessingResponse& res)
{
  framefab_path_post_processing::PathPostProcessor path_pprocessor;
  path_pprocessor.setParams(req.model_params, req.path_params);

  switch (req.action)
  {
    case framefab_msgs::PathPostProcessing::Request::PROCESS_PATH_AND_MARKER:
    {
      if(!(path_pprocessor.createCandidatePoses() && path_pprocessor.createEnvCollisionObjs()))
      {
        ROS_ERROR("Could not post process input path!");
        res.succeeded = false;
        return false;
      }
      break;
    }

    default:
    {
      ROS_ERROR("Unrecognized path post processing request");
      res.succeeded = false;
      return false;
    }
  }

  std::vector<framefab_utils::UnitProcessPath> path_array =
      path_pprocessor.getCandidatePoses();

  std::vector<moveit_msgs::CollisionObject> env_objs =
      path_pprocessor.getEnvCollisionObjs();

  for(auto unit_path : path_array)
  {
    res.process.push_back(unit_path.asElementCandidatePoses());
  }

  for(auto env_obj : env_objs)
  {
    res.env_collision_objs.push_back(env_obj);
  }

  res.succeeded = true;
  return true;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "path_post_processor");
  ros::NodeHandle nh;

  ros::ServiceServer path_post_process_server =
      nh.advertiseService<framefab_msgs::PathPostProcessingRequest, framefab_msgs::PathPostProcessingResponse>(
          "path_post_processing", boost::bind(processPathCallback, _1, _2));

  ROS_INFO("%s ready to service requests.", path_post_process_server.getService().c_str());
  ros::spin();

  return 0;
}
