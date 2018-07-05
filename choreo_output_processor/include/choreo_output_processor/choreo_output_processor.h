//
// Created by yijiangh on 9/10/17.
//

#ifndef CHOREO_OUTPUT_PROCESSOR_CHOREO_OUTPUT_PROCESSOR_H
#define CHOREO_OUTPUT_PROCESSOR_CHOREO_OUTPUT_PROCESSOR_H

// msg
#include <choreo_msgs/UnitProcessPlan.h>

namespace choreo_output_processor
{

class OutputProcessor
{
 public:
  OutputProcessor() {}
  virtual ~OutputProcessor() {}

  void setSaveFilePath(std::string &file_path) { save_file_path_ = file_path; }
  bool outputJson(std::vector<choreo_msgs::UnitProcessPlan> &plans);

 private:
  std::string save_file_path_;
};
}

#endif //CHOREO_OUTPUT_PROCESSOR_CHOREO_OUTPUT_PROCESSOR_H
