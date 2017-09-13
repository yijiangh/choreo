//
// Created by yijiangh on 9/10/17.
//

#ifndef FRAMEFAB_OUTPUT_PROCESSOR_FRAMEFAB_OUTPUT_PROCESSOR_H
#define FRAMEFAB_OUTPUT_PROCESSOR_FRAMEFAB_OUTPUT_PROCESSOR_H

// msg
#include <framefab_msgs/UnitProcessPlan.h>

namespace framefab_output_processor
{

class OutputProcessor
{
 public:
  OutputProcessor() {}
  virtual ~OutputProcessor() {}

  void setSaveFilePath(std::string &file_path) { save_file_path_ = file_path; }
  bool outputJson(std::vector<framefab_msgs::UnitProcessPlan> &plans);

 private:
  std::string save_file_path_;
};
}

#endif //FRAMEFAB_OUTPUT_PROCESSOR_FRAMEFAB_OUTPUT_PROCESSOR_H
