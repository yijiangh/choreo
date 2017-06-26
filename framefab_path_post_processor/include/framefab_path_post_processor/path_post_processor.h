//
// Created by yijiangh on 6/25/17.
//

#ifndef FRAMEFAB_PATH_POST_PROCESSOR_PATH_POST_PROCESSOR_H
#define FRAMEFAB_PATH_POST_PROCESSOR_PATH_POST_PROCESSOR_H

namespace framefab_path_post_processing
{

class PathPostProcessor
{

 public:
  PathPostProcessor();
  virtual ~PathPostProcessor() {};

  bool createCandidatePoses();
  const ElementCandidatePoses& getCandidatePoses() const { return candidate_poses; }

  // data setting
  void setPathInputParams(framefab_msgs::PathInputParameters);

 private:
  framefab_msgs::PathInputParameters path_input_params_;

};

}

#endif //FRAMEFAB_PATH_POST_PROCESSOR_PATH_POST_PROCESSOR_H
