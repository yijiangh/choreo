/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	StiffnessSolver
*
*		Description:
*
*		Version:  1.0
*		Created:  Mar/23/2016
*		Updated:  Aug/24/2016
*
*		Author:  Xin Hu, Yijiang Huang, Guoxian Song
*		Company:  GCL@USTC
*		Citation:	this module utilize solver and data structure from
*			Title:			Eigen
*							C++ template library for linear algebra: matrices, vectors, numerical 
*							solvers, and related algorithms.
*			Code Version:	3.2.9
*			Availability:	http://eigen.tuxfamily.org/index.php?title=Main_Page
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

#ifndef STIFFNESS_SOLVER_H
#define STIFFNESS_SOLVER_H

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/LU>

#include "choreo_task_sequence_planner/utils/Timer.h"

#define sind(x) (sin(fmod((x),360) * M_PI / 180))
#define cosd(x) (cos(fmod((x),360) * M_PI / 180))

using namespace std;

class StiffnessSolver
{
public:
	typedef Eigen::SparseMatrix<double> SpMat;
	typedef Eigen::MatrixXd				MX;
	typedef Eigen::Matrix3d				M3;
	typedef Eigen::VectorXd				VX;
	typedef Eigen::Vector3d				V3;
	typedef Eigen::VectorXi				VXi;
	typedef	Eigen::MatrixXi				MXi;
	
public:
	StiffnessSolver();
	~StiffnessSolver(){};

public:
	/* solver I/O*/

    /*
    * SolveSystem  -  solve {F} =   [K]{D} via L D L' decomposition        2/Dec/2015
    *                 This override function is implemented for sovling Kqq * d_q = F_q,
    *                 where no partitioned LDLt is needed.
    * @param K   : stiffness matrix for the restrained frame
    * @param D   : displacement vector to be solved
    * @param F   : mechenical force(external load vector)
    * @param verbose	:  =1 for copious screenplay
    * @param info: <0 : not stiffness matrix positive definite
    * Note: This function use eigen library SimplicialLDLt module to solve the linear system.
    */
    bool SolveSystem(
        SpMat &K,
		VX &D, 
		VX &F, 
		int verbose, 
		int &info  
        );


	/*
	* SolveSystem  -  solve {F} =   [K]{D} via conjugate gradient with guess       30/Mar/2016
	*                 This override function is implemented for sovling Kqq * d_q = F_q,
	*                 where no partitioned LDLt is needed.
	* @param K   : stiffness matrix for the restrained frame
	* @param D   : displacement vector to be solved
	* @param F   : mechenical force(external load vector)
	* @param verbose	:  =1 for copious screenplay
	* @param info: <0 : not stiffness matrix positive definite
	* Note: This function use eigen library :ConjugateGradient module to solve the linear system.
	*/
	bool SolveSystem(
		SpMat &K, 
		VX &D, 
		VX &F, 
		VX &D0, 
		int verbose, 
		int &info
		);

	void Debug();

public:
	/*
	* This LUDecomp module use Eigen library to solve the linear system
	*/
	bool LUDecomp(
		MX &A,
		VX &x,
		VX &b
		);

public:
	Timer	compute_k_;
	Timer	solve_d_;

	/* Timing Stat */
	bool		detailed_timing_;
};

#endif