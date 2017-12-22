/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	SequenceAnalyzer
*
*		Description:  perform tool path searching algorithm to generate
*				a collision-free, structurally stable path.
*
*		Version: 2.0
*		Created: Oct/10/2015
*		Updated: Aug/24/2016
*
*		Author:  Xin Hu, Yijiang Huang, Guoxian Song
*		Company:  GCL@USTC
*		Successor:			FFAnalyzer - FrameFab sequence analyzer
*							BFAnalyzer - Brute Force sequence analyzer

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

#include "framefab_task_sequence_planner/utils/QuadricCollision.h"
#include "framefab_task_sequence_planner/utils/ResolveAngle.h"

class SeqAnalyzer
{
public:
	typedef Eigen::MatrixXd MX;
	typedef Eigen::Matrix3d M3;
	typedef Eigen::VectorXd VX;
	typedef Eigen::Vector3d V3;

public:
	SeqAnalyzer();
	SeqAnalyzer(
		DualGraph			*ptr_dualgraph,
		QuadricCollision	*ptr_collision,
		Stiffness			*ptr_stiffness,
		FiberPrintPARM		*ptr_parm,
		char				*ptr_path,
		bool				terminal_output = false,
		bool				file_output = false
		);
	virtual ~SeqAnalyzer();

public:
	virtual bool	SeqPrint();
	virtual void	PrintOutTimer();

public:
	bool			InputPrintOrder(vector<int> &print_queue);
	void			OutputPrintOrder(vector<WF_edge*> &print_queue);

protected:
	void			Init();

	void			PrintPillars();
	void			UpdateStructure(WF_edge *e);
	void			RecoverStructure(WF_edge *e);
	void			UpdateStateMap(WF_edge *e, vector<vector<lld>> &state_map);
	void			RecoverStateMap(WF_edge *e, vector<vector<lld>> &state_map);
	bool			TestifyStiffness(WF_edge *e);

public:
	WireFrame			*ptr_frame_;
	DualGraph			*ptr_dualgraph_;
	Stiffness			*ptr_stiffness_;
	QuadricCollision	*ptr_collision_;
	char				*ptr_path_;

protected:
	/* maintaining for sequence */
	int							Nd_;
	DualGraph					*ptr_wholegraph_;
	vector<WF_edge*>			print_queue_;
	vector<vector<lld>>			angle_state_;
	VX							D0_;

	/* parameters */
	double				gamma_;						// gamma_	: amplifier factor for adjacency cost
	double				D_tol_;						// Dt_tol	: tolerance of offset in stiffness
	double				Wp_;						// Wp_		: stablity weight for printing cost
	double				Wa_;						// Wa_		: adjacent weight for printing cost
	double				Wi_;						// Wl_		: influence weight for printing cost

	/* for debuging */
	bool				terminal_output_;
	bool				file_output_;

	Timer				upd_struct_;
	Timer				rec_struct_;
	Timer				upd_map_;
	Timer				upd_map_collision_;
	Timer				rec_map_;
	Timer				test_stiff_;
};
