//
// Created by yijiangh on 7/5/17.
//

#ifndef FRAMEFAB_CORE_TRAJECTORY_LIBRARY_H
#define FRAMEFAB_CORE_TRAJECTORY_LIBRARY_H

#include <string>
#include <map>

#include <choreo_msgs/UnitProcessPlan.h>

namespace choreo_core_service
{

class TrajectoryLibrary
{
 public:
  typedef std::map<std::string, choreo_msgs::UnitProcessPlan> TrajectoryMap;

  void load(const std::string& filename);
  void save(const std::string& filename);

  TrajectoryMap& get() { return map_; }
  const TrajectoryMap& get() const { return map_; }

 private:
  TrajectoryMap map_;
};
}

#endif //FRAMEFAB_CORE_TRAJECTORY_LIBRARY_H
