/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	QuadricCollision
*
*		Description:
*
*		Version:  2.0
*		Created:  Oct/20/2015
*		Updated: Aug/24/2016
*
*		Author:  Xin Hu, Yijiang Huang, Guoxian Song
*		Company:  GCL@USTC
*		Citation:	This file use some geometric API and objects from
*			Title:			Geometric Tools Engine
*							a library of source code for computing in the fields of
*							mathematics, graphics, image analysis, and physics.
*			Code Version:	3.2.6
*			Availability:	http://www.geometrictools.com/index.html
----------------------------------------------------------------------------
*		Copyright (C) 2016  Yijiang Huang, Xin Hu, Guoxian Song, Juyong Zhang
*		and Ligang Liu.
*
*		FrameFab is free software: you can redistribute it and/or modify
*		it under the terms of the GNU General Public License as published by
*		the Free Software Foundation, either version 3 of the License, or
*		(at your option) any later version.
*
*		FrameFab is distributed in the hope that it will be useful,
*		but WITHOUT ANY WARRANTY; without even the implied warranty of
*		MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*		GNU General Public License for more details.
*
*		You should have received a copy of the GNU General Public License
*		along with FrameFab.  If not, see <http://www.gnu.org/licenses/>.
* ==========================================================================
*/

#ifndef FRAMEFAB_TASK_SEQUENCE_PLANNER_QUADCOLLISION_H
#define FRAMEFAB_TASK_SEQUENCE_PLANNER_QUADCOLLISION_H

#include "choreo_task_sequence_planner/utils/GCommon.h"
#include "choreo_task_sequence_planner/utils/Geometry.h"
#include "choreo_task_sequence_planner/utils/ExtruderCone.h"
#include "choreo_task_sequence_planner/utils/Triangle.h"
#include "choreo_task_sequence_planner/utils/WireFrame.h"

// geometric tools engine
#include <GTEnginePCH.h>

#include <Eigen/Core>

using namespace Geometry;
using namespace std;

typedef std::array<bool, DIR_SPHERE_DIVISION> EEDirArray;

class QuadricCollision
{

 public:
  QuadricCollision();
  ~QuadricCollision() {}

  // angle-id conversion
  // https://en.wikipedia.org/wiki/Spherical_coordinate_system
  // ISO naming convention (commonly used in physics)
  // theta \in (0, pi), phi \in (0, 2pi)
  // polar angle theta (rad)int angleToCMapId(const double phi, const double theta);
  void cmapIDToAngle(const int id, double& phi, double& theta);
  int angleToCMapId(const double phi, const double theta);

  // assuming exist_e is static collision obj, check target_e's extrusion direction map
  bool DetectCollision(const WF_edge* target_e, const WF_edge* exist_e, EEDirArray& res_cmap);

  bool DetectCollision(const WF_edge* target_e, const WF_edge* exist_e,
                       const EEDirArray& target_cmap, EEDirArray& res_cmap);

  // get initial collision map
  EEDirArray getInitCollisionMap();

  // TODO: should make into a class-free helper functions
  void ModifyAngle(const EEDirArray& cmap, EEDirArray& impacted_cmap);
  std::vector<Eigen::Vector3d> ConvertCollisionMapToEigenDirections(const EEDirArray& cmap);

 private:
  bool DetectBulk(const WF_edge* target_e, const WF_edge* exist_e,
                  const double phi, const double theta);

  // all the collision checking cases...
  bool DetectAngle(GeoV3 connect, GeoV3 end, GeoV3 target_end, GeoV3 normal);
  bool Case(GeoV3 target_start, GeoV3 target_end,
            GeoV3 order_start, GeoV3 order_end, GeoV3 normal);
  bool SpecialCase(GeoV3 connect, GeoV3 target_s, GeoV3 order_s, GeoV3 normal);
  bool ParallelCase(GeoV3 target_start, GeoV3 target_end,
                    GeoV3 order_start, GeoV3 order_end, GeoV3 normal);

  bool DetectCone(GeoV3 start, GeoV3 normal, GeoV3 target_start, GeoV3 target_end);
  bool DetectCylinder(GeoV3 start, GeoV3 normal, GeoV3 target_start, GeoV3 target_end);
  bool DetectTriangle(Triangle triangle, GeoV3 target_start, GeoV3 target_end);
  bool DetectTopCylinder(GeoV3 start, GeoV3 normal, GeoV3 target_start, GeoV3 target_end);

  void GenerateVolume(GeoV3 start, GeoV3 end, GeoV3 target_start, GeoV3 target_end, GeoV3 normal);
  void GenerateVolume(GeoV3 connect, GeoV3 target_s, GeoV3 order_s, GeoV3 normal);

 private:
  ExtruderCone extruder_;
  std::vector<Triangle> bulk_;

  EEDirArray init_cmap_;
};

#endif