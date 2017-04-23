/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	CoordTrans
*
*		Description:	Local -> global coordinate transformation module used in stiffness matrix
*						formulation process.
*
*		Version:  1.0
*		Created:  Mar/23/2016
*		Update :  Mar/25/2016
*
*		Author:  Xin Hu, Yijiang Huang, Guoxian Song
*		Company:  GCL@USTC
*		Citation:	This file is modified out of coordtrans.c from
*			Title:			Frame3dd source code
*							Static and dynamic structural analysis of 2D and 3D frames and trusses with
*							elastic and geometric stiffness.
*			Author:			Henri P. Gavin
*			Code Version:	20140514+
*			Availability:	http://frame3dd.sourceforge.net/
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
#include <vector>
#include <Eigen/Dense>

#include "framefab/WireFrame.h"

/**
CoordTrans -  evaluate the 3D coordinate transformation coefficients
Default order of coordinate rotations...  typical for Y as the vertical axis
1. rotate about the global Z axis
2. rotate about the global Y axis
3. rotate about the local  x axis --- element 'roll'

If Zvert is defined as 1, then the order of coordinate rotations is typical
for Z as the vertical axis
1. rotate about the global Y axis
2. rotate about the global Z axis
3. rotate about the local  x axis --- element 'roll'

Q=TF;   U=TD;   T'T=I;   Q=kU;   TF=kTD;   T'TF=T'kTD;   T'kT = K;   F=KD
*/
class CoordTrans{
public:
	typedef		Eigen::MatrixXd MX;
	typedef		Eigen::Matrix3d M3;
	typedef		Eigen::VectorXd VX;
	typedef		Eigen::Vector3d V3;
	typedef		Eigen::VectorXi VXi;
	typedef		Eigen::MatrixXi MXi;
public:
	CoordTrans(){};
	~CoordTrans(){};

	void CreateTransMatrix(
		std::vector<V3>	xyz,
		double L,			// length of the element(edge)
		int n1, int n2,		// index fo endpoint of the element
		double &t0, double &t1, double &t2, double &t3, double &t4, 
		double &t5, double &t6, double &t7, double &t8,
		float p);

	void CreateTransMatrix(
		point u, point v,
		double &t0, double &t1, double &t2, double &t3, double &t4,
		double &t5, double &t6, double &t7, double &t8,
		float p);

	void TransLocToGlob(
		double t0, double t1, double t2, double t3, double t4,
		double t5, double t6, double t7, double t8,
		MX	   &m, float r1, float r2);
};