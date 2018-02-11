/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	Geometry
*
*		Description:  this file defines some basic geomerty data structure and operation.
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

#include <cmath>
#include <iostream>
#include "framefab_task_sequence_planner/utils/GCommon.h"
#include "framefab_task_sequence_planner/utils/WireFrame.h"

using std::cout;

namespace Geometry
{
	//basic geomertical settings
	// the toleration of error

	/* upper bound */
	static double inf = 10000000;

	/* lower bound */
	static double ninf = -10000000;

	/* Vector class in 3 dimension euclidean space */
	class Vector3d
	{
	public:
		Vector3d(double x = 0, double y = 0, double z = 0)
		{
			data_[0] = x;
			data_[1] = y;
			data_[2] = z;
		}

		Vector3d(point o)
		{
			data_[0] = o.x();
			data_[1] = o.y();
			data_[2] = o.z();
		}

		Vector3d(point start, point end)
		{
			data_[0] = end.x()-start.x();
			data_[1] = end.y()-start.y();
			data_[2] = end.z()-start.z();
		}
		~Vector3d(){}

	public:
		double getX(){ return data_[0]; }
		double getY(){ return data_[1]; }
		double getZ(){ return data_[2]; }
		double *data(){ return data_; }

	public:
		Vector3d operator+ (const Vector3d &b)
		{
			return Vector3d(data_[0] + b.data_[0], data_[1] + b.data_[1], data_[2] + b.data_[2]);
		}

		Vector3d operator- (const Vector3d &b)
		{
			return Vector3d(data_[0] - b.data_[0], data_[1] - b.data_[1], data_[2] - b.data_[2]);
		}

		Vector3d operator* (double k)
		{
			return Vector3d(data_[0] * k, data_[1] * k, data_[2] * k);
		}

		double operator[] (int k)
		{
			if (0 <= k && k < 3)
				return data_[k];
			else
				return 0;
		}

	public:
		double norm()
		{
			return sqrt(data_[0] * data_[0] + data_[1] * data_[1] + data_[2] * data_[2]);
		}
		void normalize()
		{
			double length = norm();
			if (length > GEO_EPS)
			{
				for (int i = 0; i < 3; i++)
					data_[i] /= length;
			}
			else
			{
				cout << "error:zero vector cannnot be normalized";
			}
		}

	private:
		double data_[3];
	};

	/* For Vector3d(dot product) */
	static Vector3d cross(Vector3d vec1, Vector3d vec2)
	{
		double u[3] = { vec1.getX(), vec1.getY(), vec1.getZ() };
		double v[3] = { vec2.getX(), vec2.getY(), vec2.getZ() };
		return Vector3d(u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]);
	}

	/* For Vector3d(cross product) */
	static double dot(Vector3d vec1, Vector3d vec2)
	{
		double u[3] = { vec1.getX(), vec1.getY(), vec1.getZ() };
		double v[3] = { vec2.getX(), vec2.getY(), vec2.getZ() };
		return u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
	}

	static double angle(Vector3d vec1, Vector3d vec2)
	{
		double temp = 0;
		temp = dot(vec1, vec2) / (vec1.norm()*vec2.norm());
		//--------------------------
		if (abs(temp - 1) < GEO_EPS)
			return 0;
		if (abs(temp + 1) < GEO_EPS)
			return 3.1415;

		return acos(temp);
	}

	/* For Vector3d
	* suppose i = (1,0,0), j = (0,1,0),k = (0,0,1),O =(0,0,0)
	* and point is in the coordinate system E1={O;i,j,k}
	* and we have another coordinate system E2={O;coord[1],coord[2],coord[3]};
	* then we can use the point's coordination in E1 to compute the point's coordination in the E2
	*/

	static void changeCoordinate(Vector3d &point, Vector3d coord[3])
	{
		double result[3] = { 0, 0, 0 };
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				result[i] += point[j] * (coord[i])[j];
			}
		}
		point = Geometry::Vector3d(result[0], result[1], result[2]);
	}

	static void changeCoordinate(Vector3d &point, Vector3d coord[3], Vector3d origin)
	{
		double result[3] = { 0, 0, 0 };
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				result[i] += point[j] * (coord[i])[j];
			}
		}
		point = Geometry::Vector3d(result[0], result[1], result[2]);
		point = point - origin;
	}
}