/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	ResolveAngle
*
*		Description:
*
*		Version:  2.0
*		Created:  Oct/20/2015
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

#pragma once

#include "choreo_task_sequence_planner/utils/WireFrame.h"
#include "choreo_task_sequence_planner/utils/Geometry.h"
#include "choreo_task_sequence_planner/utils/ExtruderCone.h"

class ResolveAngle
{
 public:
  ResolveAngle();
  ResolveAngle( vector< Geometry::Vector3d> list );
  ~ResolveAngle();

  Geometry::Vector3d dec;
  double wave;
  vector< Geometry::Vector3d> a_;
  vector< Geometry::Vector3d> b_;
  vector< Geometry::Vector3d> c_;

  ExtruderCone extruder_;
  vector< Geometry::Vector3d> list_;
  vector< Geometry::Vector3d> Resolve();

  void Dec();

  Geometry::Vector3d Ave(vector<Geometry::Vector3d> t);
};

