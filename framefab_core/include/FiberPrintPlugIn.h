/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	FiberPrintPlugin
*
*		Description:	This module is a container for several searching and cut computational
*		module, which are public slots to renderwidgets.
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

#ifndef FIBERPRINTPLUGIN_H
#define FIBERPRINTPLUGIN_H

#include <vector>
#include "framefab/ADMMCut.h"
#include "framefab/NormalCut.h"
#include "framefab/FFAnalyzer.h"
#include "framefab/BFAnalyzer.h"
#include "framefab/ProcAnalyzer.h"


class FiberPrintPlugIn
{
public:
	typedef Eigen::MatrixXd MX;
	typedef Eigen::VectorXd VX;

public:
	FiberPrintPlugIn();
	FiberPrintPlugIn(
		WireFrame *ptr_frame, FiberPrintPARM *ptr_parm, char *ptr_path = NULL, 
		bool terminal_output = false, bool file_output = false
		);
	~FiberPrintPlugIn();

public:
	void			Init();

	/* Fiber printing */
	void			FrameFabPrint();
	void			BruteForcePrint();
	void			SweepingPrint();
	void			OneLayerPrint();

	/* apply stiffness computation directly to the input frame shape */
	void			GetDeformation();
	void			GetFrameFabCut();

	bool			ImportPrintOrder(char *fname);
	void			ExportPrintOrder(char *fname);

	void			InputPrintOrder(vector<int> &queue)			{ ptr_seqanalyzer_->InputPrintOrder(queue); }
	void			OutputPrintOrder(vector<WF_edge*> &queue)	{ ptr_seqanalyzer_->OutputPrintOrder(queue); }
	void			ExportRenderPath(int min_layer, int max_layer, char *ptr_path)
	{
		ptr_seqanalyzer_->WriteRenderPath(min_layer, max_layer, ptr_path);
	}

	void			Debug();		// return value: edge index in mesh, for cut rendering

public:
	WireFrame			*ptr_frame_;
	DualGraph			*ptr_dualgraph_;
	QuadricCollision	*ptr_collision_;
	Stiffness			*ptr_stiffness_;

	GraphCut			*ptr_graphcut_;
	SeqAnalyzer			*ptr_seqanalyzer_;
	ProcAnalyzer		*ptr_procanalyzer_;

	char				*ptr_path_;
	FiberPrintPARM		*ptr_parm_;

private:
	Timer				fiber_print_;

	bool				terminal_output_;
	bool				file_output_;
};

#endif // FIBERPRINTPLUGIN_H
