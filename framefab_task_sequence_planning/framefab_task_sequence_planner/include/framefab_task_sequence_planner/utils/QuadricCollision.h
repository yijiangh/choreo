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

#include "framefab_task_sequence_planner/utils/Geometry.h"
#include "framefab_task_sequence_planner/utils/ExtruderCone.h"
#include "framefab_task_sequence_planner/utils/Triangle.h"
#include "framefab_task_sequence_planner/utils/WireFrame.h"
#include "framefab_task_sequence_planner/utils/DualGraph.h"

// geometric tools engine
#include <GTEnginePCH.h>
#include <Mathematics/GteTriangle.h>
#include <Mathematics/GteIntrSegment3Cone3.h>
#include <Mathematics/GteIntrSegment3Cylinder3.h>
#include <Mathematics/GteIntrSegment3Triangle3.h>
#include <Mathematics/GteDistSegmentSegment.h>

#define	STRICT_COLLISION

// used to express feasible end effector directions
typedef unsigned long long lld;

using namespace Geometry;
using namespace std;

// theta=(0,180), phi=(0,360)
// target means unprinted edge under current consideration, order edge means existing edge

class QuadricCollision
{
 public:
  QuadricCollision();
  QuadricCollision(WireFrame *ptr_frame);
  ~QuadricCollision();

 public:
  void Init(vector <lld> &colli_map);
  bool DetectCollision(WF_edge* target_e, DualGraph* ptr_subgraph, std::vector<lld>& result_map);
  bool DetectCollision(WF_edge* target_e, WF_edge* order_e, std::vector<lld>& colli_map);
  void DetectCollision(WF_edge* target_e, std::vector<WF_edge*> exist_edge, std::vector<GeoV3>& output);

 public:
  void ModifyAngle(std::vector<lld>& angle_state, const std::vector<lld>& colli_map);

  int ColFreeAngle(const std::vector<lld>& colli_map);

  std::vector<Eigen::Vector3d> ConvertCollisionMapToEigenDirections(const std::vector<lld>& colli_map);
  std::vector<int> ConvertCollisionMapToIntMap(const std::vector<lld>& colli_map);

  inline int Divide()
  {
    return 18 * 10 + 2;
  }

 private:
  // convert to eef direction 3d vector
  inline GeoV3 Orientation(double theta, double phi)
  {
    return GeoV3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
  }

  inline Eigen::Vector3d ConvertAngleToEigenDirection(double theta, double phi)
  {
    return Eigen::Vector3d(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
  }

 private:
  bool DetectBulk(WF_edge *order_e, double theta, double phi);
  bool DetectEdge(WF_edge *order_e, vector <lld> &colli_map);
  bool DetectEdges(std::vector<WF_edge*> exist_edge, double theta, double phi);
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

  bool Parallel(GeoV3 a, GeoV3 b);
  double Distance(WF_edge *order_e);

  gte::Segment<3, float> Seg(point target_start, point target_end);
  gte::Segment<3, float> Seg(GeoV3 target_start, GeoV3 target_end);
  gte::Triangle<3, float> Tri(GeoV3 a, GeoV3 b, GeoV3 c);

 public:
  WireFrame* ptr_frame_;
  WF_edge* target_e_;

 private:
  ExtruderCone extruder_;
  std::vector<Triangle> bulk_;
  int divide_;

  // compact representation of end effector's feasible direction
  // each lld is a bit-wise map, value 1 means it causes collision
  std::vector<std::vector<lld>*> colli_map_;
};

#endif