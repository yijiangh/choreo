/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	ADMMCut
*
*		Description:	The GraphCut submodule takes charge of dividing the
*				wireframe into several structually-stable sections, scaling the
*				problem down, enabling further tool path searching part
*				numerically tractable.
*
*		Version:  2.0
*		Created: Oct/10/2015
*		Updated: Aug/24/2016
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

#include <omp.h>
#include <iostream>

// eigen
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>
#include <Eigen/Core>

// utils
#include <utils/GCommon.h>
#include <utils/Statistics.h>

// cut_generators
#include <cut_generators/QPMosek.h>
#include <cut_generators/QPFactory.h>
#include <cut_generators/GraphCut.h>

using namespace std;
using namespace Eigen;


class ADMMCut : public GraphCut
{
	typedef Eigen::SparseMatrix<double> SpMat;
	typedef Eigen::MatrixXd MX;
	typedef Eigen::Matrix3d M3;
	typedef Eigen::VectorXd VX;
	typedef Eigen::Vector3d V3;

public:
	ADMMCut();
	ADMMCut(
		DualGraph			*ptr_dualgraph,
		QuadricCollision	*ptr_collision,
		Stiffness			*ptr_stiffness,
		FiberPrintPARM		*ptr_parm,
		char				*ptr_path,
		bool				terminal_output = false, 
		bool				file_output = false
		);
	~ADMMCut();

public:
	void		MakeLayers();						// Main loop of cut

private:
	void		InitState();						// Initialization
	void		InitWeight();

	void		SetStartingPoints();				// Set D and lambda variable's starting value
	void		SetBoundary();
	void		CreateA();
	void		CalculateX();						// QP optimization for x at every iteration
	void 		CalculateQ(const VX _D, SpMat &Q);	// Calculate Q for x_Qp problem
	void		CalculateD();						// QP optimization for D at every iteration
	void		CalculateY();						// Direct quadratic optimization for Y at every iteration
	void		UpdateLambda();						// Dual variable update at every iteration
	void		UpdateCut();
	bool		UpdateR(VX &x_prev);

	bool		CheckLabel();						// Stopping Criterion for iteratively apply ADMM to find several cuts
	bool		TerminationCriteria();				// Termination Criteria for ADMM process of a single cut using a threshold node number

	void		PrintOutTimer();
	void		WriteWeight();
	void		WriteStiffness();
	void		Debug();

public:
	DualGraph			*ptr_dualgraph_;
	Stiffness			*ptr_stiffness_;
	QuadricCollision	*ptr_collision_;

private:
	SpMat				A_;				// incidence matrix for constraints Y = E * X
	SpMat				weight_;		// for weight, indexed by half of original id
	MX					r_;				// for updation of C, indexed by half of dual id
	VX					x_;
	VX					D_;
	VX					y_;
	VX					lambda_stf_;	// dimension 6 * Ns_, corresponding to equation constraints Kd = F
	VX					lambda_y_;		// dimension 2 * Md_, corresponding to equation constraints yij = xi - xj
	VX					a_;				// linear coefficient used in x_Qp
	VX					d_;				// for setting boundary & QP x

	double				dual_res_;		// dual residual for ADMM termination criteria
	double				primal_res_;	// dual residual for ADMM termination criteria

	/* 
	Solves the quadratic programming problem:
	min 0.5* xt*H*x + ft*x subject to A*x <= b, C*x = d, x >= lb, x <= ub 
	*/
	QP					*ptr_qp_;	
	SpMat				W_;

	int					cut_round_;
	int					reweight_round_;
	int					ADMM_round_;

	int					N_;				// N :    Number of nodes in orig graph
	int					M_;				// M :    Number of edges in orig graph 
	int					Nd_;			// Nd :   Number of node in dual graph
	int					Md_;			// Md :   Number of edges in dual graph
	int					Fd_;			// Fd :   Number of faces in dual graph
	int					Ns_;
	int					Nd_w_;		    // Nd_w_: Number of nodes in WHOLE dual graph 

	int					stop_n_;		// stop_n   : termination criteria for ADMMCut process, number of dual nodes in LowerSet
	double				D_tol_;			// Dt_tol   : tolerance of offset in stiffness
	double				penalty_;		// penalty  : penalty factor used in ADMM  
	double				pri_tol_;		// pri_tol  : primal residual tolerance for ADMM termination criterion
	double				dual_tol_;		// dual_tol : dual   residual tolerance for ADMM termination criterion

	double				K_eps_;

	SpMat				K_;
	VX					F_;
	SpMat				Q_;
	SpMat				H1_;

	Timer				ADMM_cut_;
	Timer				init_state_;
	Timer				init_weight_;
	Timer				create_a_;
	Timer				set_bound_;
	Timer				set_startpoint_;
	Timer				cal_y_;
	Timer				cal_x_;
	Timer				cal_q_;
	Timer				cal_qp_;
	Timer				cal_d_;
	Timer				update_lambda_;
	Timer				update_cut_;
	Timer				update_r_;
};

