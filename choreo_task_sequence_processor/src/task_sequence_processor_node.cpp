#include <ros/ros.h>
#include <ros/console.h>

#include <boost/tuple/tuple.hpp>

#include <choreo_msgs/TaskSequenceProcessing.h>
#include <choreo_task_sequence_processor/unit_process.h>
#include <choreo_task_sequence_processor/task_sequence_processor.h>

bool processTaskSequenceCallback(choreo_msgs::TaskSequenceProcessingRequest& req,
                                 choreo_msgs::TaskSequenceProcessingResponse& res)
{
  choreo_task_sequence_processing::TaskSequenceProcessor ts_processor;

  switch (req.action)
  {
    case choreo_msgs::TaskSequenceProcessing::Request::SPATIAL_EXTRUSION:
    {
      ROS_WARN_STREAM("[ts processor] using spatial extrusion parsing mode now.");

      // TODO: ARCHIVED for now
      ts_processor.setParams(req.model_params, req.task_sequence_params, req.world_frame);

      if(!(ts_processor.createCandidatePoses() && ts_processor.createEnvCollisionObjs()))
      {
        ROS_ERROR("[Task Seq Process] Could not process input task sequence!");
        res.succeeded = false;
        return false;
      }

      std::vector<choreo_task_sequence_processing_utils::UnitProcess> process_array =
          ts_processor.getCandidatePoses();

      std::vector<moveit_msgs::CollisionObject> env_objs =
          ts_processor.getEnvCollisionObjs();

      for(auto& unit_process : process_array)
      {
        res.process.push_back(unit_process.asElementCandidatePoses());
      }

      for(auto& env_obj : env_objs)
      {
        res.env_collision_objs.push_back(env_obj);
      }

      break;
    }
    case choreo_msgs::TaskSequenceProcessing::Request::PICKNPLACE:
    {
      if(!ts_processor.parseAssemblySequencePickNPlace(
          req.model_params, req.task_sequence_params, req.world_frame, res.assembly_sequence_pnp))
      {
        ROS_ERROR("[Task Seq Process] Could not process input task sequence!");
        res.succeeded = false;
        return false;
      }

      return true;

      break;
    }
    default:
    {
      ROS_ERROR("[Task Seq Process] Unrecognized task sequence processing request");
      res.succeeded = false;
      return false;
    }
  }

  res.succeeded = true;
  return true;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "task_sequence_processor");
  ros::NodeHandle nh;

  ros::ServiceServer task_sequence_processing_server =
      nh.advertiseService<choreo_msgs::TaskSequenceProcessingRequest, choreo_msgs::TaskSequenceProcessingResponse>(
          "task_sequence_processing", boost::bind(processTaskSequenceCallback, _1, _2));

  ROS_INFO("[task seq processor] %s server ready to service requests.", task_sequence_processing_server.getService().c_str());
  ros::spin();

  return 0;
}
