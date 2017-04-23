/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	BFAnalyzer
*
*		Description:	perform Brute Force algorithm to generate a collision-free,
*				structurally-stable path.
*
*		Version:  1.0
*		Created:  Mar/23/2016
*		Update :  Mar/25/2016
*
*		Author:  Xin Hu, Yijiang Huang, Guoxian Song
*		Company:  GCL@USTC
*		Note:	Generate every possible tool path and remove paths that are
*				not collision-free or structurally-stable.
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

#include "framefab/SeqAnalyzer.h"

class BFAnalyzer : public SeqAnalyzer
{
public:
	BFAnalyzer();
	BFAnalyzer(
		DualGraph			*ptr_dualgraph,
		QuadricCollision	*ptr_collision,
		Stiffness			*ptr_stiffness,
		FiberPrintPARM		*ptr_parm,
		char				*ptr_path,
		bool				terminal_output = false,
		bool				file_output = false
		)
		:SeqAnalyzer(ptr_dualgraph, ptr_collision, ptr_stiffness,
		ptr_parm, ptr_path, terminal_output, file_output){}
	~BFAnalyzer();

public:
	bool	SeqPrint();

private:
	bool	GenerateSeq(int h, int t);
	void	PrintOutQueue(int N);	

public:
	void	PrintOutTimer();

private:
	Timer	BF_analyzer_;
};

