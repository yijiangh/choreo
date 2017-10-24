/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	Polyface
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
//#include <windows.h>

// http://kiwwito.com/installing-opengl-glut-libraries-in-ubuntu/
#include <utils/Geometry.h>

using namespace std;

typedef Geometry::Vector3d GeoV3;

class Polyface
{
public:
	Polyface(){}
	~Polyface(){}

public:
	point Trans(GeoV3 V)
	{
		point v;
		v.x() = V.getX();
		v.y() = V.getY();
		v.z() = V.getZ();
		return v;
	}

	point v0()
	{
		return vert_list_[0];
	}

	point v1()
	{
		return vert_list_[1];
	}

	point v2()
	{
		return vert_list_[2];
	}

	point v3()
	{
		return vert_list_[3];
	}

	point Normal()
	{
		return normal_;
	}

	void Normal_()
	{
		GeoV3 normal;
		normal = Geometry::cross(v1() - v0(), v2() - v0());

		if (normal.norm() == 0)
			return;
		normal.normalize();
		normal_ = Trans(normal);
	}

	virtual void				Print() {}
	virtual void				Render(WireFrame* ptr_frame, double alpha) {}

public:
	vector<point>			vert_list_;
	point						normal_;
};

