//
// Created by yijiangh on 6/25/17.
//

#ifndef FRAMEFAB_PATH_POST_PROCESSOR_PATH_POST_PROCESSOR_H
#define FRAMEFAB_PATH_POST_PROCESSOR_PATH_POST_PROCESSOR_H

#include <framefab_msgs/ElementCandidatePoses.h>
#include <framefab_msgs/PathInputParameters.h>
//#include <framefab_path_post_processor/process_path.h>

namespace framefab_path_post_processing
{

class PathPostProcessor
{
  typedef std::vector<framefab_msgs::ElementCandidatePoses> ElementCandidatePosesArray;

 public:
  PathPostProcessor() {}
  virtual ~PathPostProcessor() {}

  bool createCandidatePoses();
  const ElementCandidatePosesArray& getCandidatePoses() const { return path_array_; }

  // data setting
  void setPathInputParams(framefab_msgs::PathInputParameters in_params) { path_input_params_ = in_params; }

  bool variable_ok() const {}

 protected:
  // add printing table
  // add printing table ref pt

 private:
  framefab_msgs::PathInputParameters path_input_params_;
  ElementCandidatePosesArray path_array_;
};

}

#endif //FRAMEFAB_PATH_POST_PROCESSOR_PATH_POST_PROCESSOR_H
