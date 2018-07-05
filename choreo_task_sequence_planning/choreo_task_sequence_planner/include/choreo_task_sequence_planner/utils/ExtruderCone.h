/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	ExtruderCone
*
*		Description:  This class maintains extruder bounding cone info
*				   and OpenGL rendering interfaces.
*
*		Version: 2.0
*		Created: Oct/20/2015
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

#pragma once
#include "choreo_task_sequence_planner/utils/GCommon.h"
#include "choreo_task_sequence_planner/utils/WireFrame.h"
#include "choreo_task_sequence_planner/utils/Triangle.h"

using namespace std;
using namespace Geometry;

class ExtruderCone
{
public:
	ExtruderCone();
	ExtruderCone(
		double height, 
		point base_point, 
		Vec3f normal, 
		double angle
	);
	~ExtruderCone();

public:
	/* Data I/O */
	double Height() { return height_; }
	double Angle() { return angle_; }
	double WaveAngle() { return wave_angle_; }
	double ToolLenth() { return tool_lenth_; }
	double Radii() { return radii_; }
	double CyclinderLenth(){ return cyclinder_height_; }
	point	BasePoint() { return base_point_; }
	Vec3f	Normal()	{ return normal_; }


	/*top cylidner*/
	double TopCenter(){ return top_cylin_center_lenth_; }
	double TopLenth(){ return top_cylin_lenth_; }
	double TopRadii(){ return top_cylin_radii_; }
	/* Generate Extruder Cone */
	void GeneCone();

	/* Debug Screenplay Function */
	void	Test();

	/* Geometric transformation, for rendering purpose */
	void	RotateTri(Triangle temp);
    point	Multi(point s);
	void	Rotation(double angle, point start, point end);
	void	Rotation(GeoV3 normal, point start, point end);

	/* OpenGL Rendering Interface */
	void	Render(WireFrame* ptr_frame, double alpha);

private:
	/* Extruder Property Data */
	double angle_;
	double height_;
	double tool_lenth_;
	double radii_;
	double cyclinder_height_;

	double			 wave_angle_;
    vector<Triangle> side_;
	point			 base_point_;
	Vec3f			 normal_;

	/* Extruder Render Data */
	int				 divide_;	/* Traingle division for cone	  */
	vector<Triangle> side_end_;  /* Triangle Approx for the cone   */
	vector<point>	 top_;		/* Ploygon represent top covering */

	/* Transformation for Rendering */
	float rotate_[4][4];
	point start_,end_;

	//Top_Cylinder
	double top_cylin_center_lenth_;
	double top_cylin_lenth_;
	double top_cylin_radii_;
};