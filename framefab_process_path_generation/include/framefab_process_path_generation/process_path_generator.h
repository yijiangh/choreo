/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 *  path_generation.h
 *
 *  Created on: Feb 28, 2016
 *  Author: Bart and Jeroen
 *
 *  Modified on: June 5, 2017
 *  Author: Yijiang Huang (MIT)
 */

#ifndef FRAMEFAB_PATH_GENERATION_PATH_GENERATOR_H
#define FRAMEFAB_PATH_GENERATION_PATH_GENERATOR_H

#include <vector>
#include <Eigen/Geometry>

namespace framefab_process_path
{

//TODO: check godel's path_generator, good structure and have similar needs

class ProcessPathGenerator
{
 public:
  ProcessPathGenerator(){};
  ~ProcessPathGenerator(){};

 public:
/**
* Get a vector of poses on a straight line between two points.
* @param startPose Transformation matrix of the starting pose
* @param endPose Transformation matrix of the end pose
* @param steps Number of trajectory points on straight line
* @return Vector containing the points of the line trajectory
*/
  std::vector<Eigen::Affine3d> addLinePathPts(Eigen::Affine3d start_pose, Eigen::Affine3d en_pose, int steps);

};
}
#endif //FRAMEFAB_PATH_GENERATION_PATH_GENERATOR_H
