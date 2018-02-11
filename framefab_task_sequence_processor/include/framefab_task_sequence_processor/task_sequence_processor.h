//
// Created by yijiangh on 6/25/17.
//

#ifndef FRAMEFAB_TASK_SEQUENCE_PROCESSOR
#define FRAMEFAB_TASK_SEQUENCE_PROCESSOR

#include <framefab_msgs/ModelInputParameters.h>
#include <framefab_msgs/TaskSequenceInputParameters.h>
#include <framefab_msgs/ElementCandidatePoses.h>

#include <moveit_msgs/CollisionObject.h>

#include <framefab_task_sequence_processor/unit_process.h>

namespace framefab_task_sequence_processing
{

class TaskSequenceProcessor
{
  typedef std::vector<framefab_msgs::ElementCandidatePoses> ElementCandidatePosesArray;

 public:
  TaskSequenceProcessor();
  virtual ~TaskSequenceProcessor() {}

  bool createCandidatePoses();
  bool createEnvCollisionObjs();
  const std::vector<framefab_task_sequence_processing_utils::UnitProcess>& getCandidatePoses() const { return path_array_; }
  const std::vector<moveit_msgs::CollisionObject>&     getEnvCollisionObjs() const { return env_collision_objs_; }

  // data setting
  void setParams(framefab_msgs::ModelInputParameters model_params,
                 framefab_msgs::TaskSequenceInputParameters task_sequence_params);

 protected:
  framefab_task_sequence_processing_utils::UnitProcess createScaledUnitProcess(int index, int wireframe_id,
                                                                               Eigen::Vector3d st_pt, Eigen::Vector3d end_pt,
                                                                               std::vector<Eigen::Vector3d> feasible_orients,
                                                                               std::string type_str,
                                                                               double element_diameter,
                                                                               double shrink_length);

  void setTransfVec(const Eigen::Vector3d& ref_pt, const Eigen::Vector3d& base_center_pt, const double& scale)
  {
    transf_vec_ = (ref_pt - base_center_pt) * scale;
  }

 private:
  // params
  framefab_msgs::ModelInputParameters model_input_params_;
  framefab_msgs::TaskSequenceInputParameters path_input_params_;

  std::vector<framefab_task_sequence_processing_utils::UnitProcess> path_array_;
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
