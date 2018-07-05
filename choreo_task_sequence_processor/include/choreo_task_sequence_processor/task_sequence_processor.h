//
// Created by yijiangh on 6/25/17.
//

#ifndef FRAMEFAB_TASK_SEQUENCE_PROCESSOR
#define FRAMEFAB_TASK_SEQUENCE_PROCESSOR

#include <choreo_msgs/ModelInputParameters.h>
#include <choreo_msgs/TaskSequenceInputParameters.h>
#include <choreo_msgs/ElementCandidatePoses.h>

#include <choreo_msgs/AssemblySequencePickNPlace.h>

#include <moveit_msgs/CollisionObject.h>

#include <choreo_task_sequence_processor/unit_process.h>

namespace choreo_task_sequence_processing
{

class TaskSequenceProcessor
{
  typedef std::vector<choreo_msgs::ElementCandidatePoses> ElementCandidatePosesArray;

 public:
  TaskSequenceProcessor();
  virtual ~TaskSequenceProcessor() {}

  // data setting
  void setParams(choreo_msgs::ModelInputParameters model_params,
                 choreo_msgs::TaskSequenceInputParameters task_sequence_params,
                 std::string world_frame);

  // TODO: this can be used as a helper function, we don't need to wrap a class around it
  // Note: this function enforces the existence for all the stl files by asserting the file existence.
  bool parseAssemblySequencePickNPlace(const choreo_msgs::ModelInputParameters& model_params,
                                       const choreo_msgs::TaskSequenceInputParameters& task_sequence_params,
                                       const std::string& world_frame_,
                                       choreo_msgs::AssemblySequencePickNPlace& as_pnp);

  // DEPRECATED
  // parson json file into path arrays (unit process)
  bool createCandidatePoses();
  bool createEnvCollisionObjs();

  const std::vector<choreo_task_sequence_processing_utils::UnitProcess>& getCandidatePoses() const { return path_array_; }
  const std::vector<moveit_msgs::CollisionObject>& getEnvCollisionObjs() const { return env_collision_objs_; }

 protected:
  choreo_task_sequence_processing_utils::UnitProcess createScaledUnitProcess(int index, int wireframe_id,
                                                                               Eigen::Vector3d st_pt, Eigen::Vector3d end_pt,
                                                                               std::vector<Eigen::Vector3d> feasible_orients,
                                                                               std::string type_str,
                                                                               double element_diameter,
                                                                               double shrink_length);

  void setTransfVec(const Eigen::Vector3d& ref_pt, const Eigen::Vector3d& base_center_pt, const double& scale)
  {
    transf_vec_ = (ref_pt - base_center_pt) * scale;
  }
  // END DEPRECATED

 private:
  // params
  choreo_msgs::ModelInputParameters model_input_params_;
  choreo_msgs::TaskSequenceInputParameters path_input_params_;
  std::string world_frame_;

  std::vector<choreo_task_sequence_processing_utils::UnitProcess> path_array_;
  std::vector<moveit_msgs::CollisionObject> env_collision_objs_;

  double unit_scale_;
  double element_diameter_;
  double shrink_length_;
  Eigen::Vector3d ref_pt_;
  Eigen::Vector3d transf_vec_;

  bool verbose_;
};
}
#endif //FRAMEFAB_TASK_SEQUENCE_PROCESSOR