/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		Description:  GCommon.h provides some common configuration for fiberprint project.
*
*		Version: 2.0
*		Created: Oct/10/2015
*		Updated: Aug/24/2016
*
*		Author:  Xin Hu, Yijiang Huang, Guoxian Song
*		Company:  GCL@USTC
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

// end effector discretization meta parameter
static const int THETA_DISC = 10;
static const int PHI_DISC = 20;
static const int DIR_SPHERE_DIVISION = 200;

#define FILENMAX 128

#ifndef F_PI
#define F_PI 3.14159265358979323846264338327950288419716939937510
#endif

#ifndef SPT_EPS
#define SPT_EPS  0.0000001	// sparse matrix eps
#endif

#ifndef GEO_EPS			// geometry eps
#define GEO_EPS  0.001
#endif

#ifndef STIFF_TOL
#define STIFF_TOL 1.0e-9	// tolerance for stiffness RMS error
#endif

#ifndef MCOND_TOL
#define MCOND_TOL 1.0e12	// tolerance for stiffness matrix condition number
#endif

// Zvert=1: Z axis is vertical... rotate about Y-axis, then rotate about Z-axis
// Zvert=0: Y axis is vertical... rotate about Z-axis, then rotate about Y-axis
#define Zvert 1

#endif /* FIBERPRINT_COMMON_H */

