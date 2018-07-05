/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	Triangle
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
#include "choreo_task_sequence_planner/utils/Polyface.h"

class Triangle : public Polyface
{
public:
	Triangle()
	{
		vert_list_.push_back(point(1, 0, 0));
		vert_list_.push_back(point(0, 1, 0));
		vert_list_.push_back(point(0, 0, 1));
		Normal_();
	}

	Triangle(point v0, point v1, point v2)
	{
		vert_list_.push_back(v0);
		vert_list_.push_back(v1);
		vert_list_.push_back(v2);
		Normal_();
	}

	Triangle(GeoV3 v0, GeoV3 v1, GeoV3 v2)
	{
		vert_list_.push_back(Trans(v0));
		vert_list_.push_back(Trans(v1));
		vert_list_.push_back(Trans(v2));
		Normal_();
	}

	~Triangle() {}

public:
	void Print()
	{
		point _v0 = v0();
		point _v1 = v1();
		point _v2 = v2();

		std::cout << _v0.x() << ", " << _v0.y() << ", " << _v0.z() << std::endl;
		std::cout << _v1.x() << ", " << _v1.y() << ", " << _v1.z() << std::endl;
		std::cout << _v2.x() << ", " << _v2.y() << ", " << _v2.z() << std::endl;
	}

	void Render(WireFrame* ptr_frame, double alpha)
	{
//		glBegin(GL_TRIANGLES);
//		glColor4f(1.0, 1.0, 0, alpha);
//		glNormal3fv(normal_);
//		glVertex3fv(ptr_frame->Unify(v0()));
//		glVertex3fv(ptr_frame->Unify(v1()));
//		glVertex3fv(ptr_frame->Unify(v2()));
//		glEnd();
	}

	void Add(point base)
	{
		for (int i = 0; i < vert_list_.size(); i++)
		{
			vert_list_[i] += base;
		}
	}
};

