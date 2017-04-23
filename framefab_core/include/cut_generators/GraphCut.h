/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		Class:	GraphCut
*
*		Description:	Cut input model into layers by different means.
*
*		Version: 2.0
*		Created: Oct/10/2015
*		Updated: Aug/24/2016
*
*		Author:  Xin Hu, Yijiang Huang, Guoxian Song
*		Company:  GCL@USTC
*		Successor:	ADMMCut		- FrameFab ADMM cut
*					NormalCut	- cut layers by sweeping from bottom to top.
*					NoneCut		- no layers
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

#include <iostream>

#include <utils/WireFrame.h>
#include <utils/Stiffness.h>
#include <utils/QuadricCollision.h>
#include <FiberPrintPARM.h>

using namespace std;


class GraphCut
{
public:
	GraphCut();
	virtual ~GraphCut();

public:
	virtual void	MakeLayers();	
	virtual void	PrintOutTimer();

public:
	WireFrame		*ptr_frame_;
	char			*ptr_path_;


	vector<int>		cutting_edge_;

protected:
	/* for debuging */
	bool			terminal_output_;
	bool			file_output_;
};

