/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		Class:	IllConditioner
*
*		Description: Matrix condition checking related computational units.
*
*		Version: 2.0
*		Created: Oct/10/2015
*		Updated: Aug/24/2016
*
*		Author:  Xin Hu, Yijiang Huang, Guoxian Song
*		Company:  GCL@USTC
*		Citation:	This file use matrix operation module from
*			Title:			LAPACK
*							a linear algebra package
*			Code Version:	3.6.1
*			Availability:	http://www.netlib.org/lapack/
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

#ifndef ILLCONDDECTOR_H
#define ILLCONDDECTOR_H

#include <iostream>
#include <stdlib.h>
#include "assert.h"
#include <Eigen/Sparse>

#include <utils/Statistics.h>

extern "C" void dgesv_(const int *N, const int *nrhs, double *A, const int *lda, int
	*ipiv, double *b, const int *ldb, int *info);

extern "C" void dgels_(const char *trans, const int *M, const int *N, const int *nrhs,
	double *A, const int *lda, double *b, const int *ldb, double *work,
	const int * lwork, int *info);

// Computes the Cholesky factorization of a real symmetric
// positive definite matrix A.
// A = Ut * U, for uplo = "U"
// A = L * Lt, for uplo = "L"
extern "C" void dpotrf_(const char *UPLO, const int *N, double *A, const int *LDA, int *info);

// Estimates the reciprocal of the condition number (in the 1 - norm)
// of a real symmetric positive definite packed matrix using
// the Cholesky factorization A = U**T*U or A = L*L**T computed by
// DPPTRF.
// refer : http://www.netlib.org/lapack/explore-html/d0/d9b/dppcon_8f.html#a8107a68e3c7d948fe246bf0feae0470b
extern "C" void dpocon_(const char *UPLO, const int *N, const double *AP, const int *lda, const double *ANORM,
	double *RCOND, double *WORK, int *LWORK, int *INFO);

// Rank computing, the rank of input matrix is the number of singular values that are not zero
// A = U * SIGMA * transpose(V)
// refer : http://icl.cs.utk.edu/lapack-forum/viewtopic.php?f=2&t=818
extern "C" void dgesvd_(const char *JOBU,  const char *JOBVT, const int *M, const int *N, double *A, const int *LDA,
	double *S, double *U, const int *LDU, double *VT, const int *LDVT, double *WORK, const int *LWORK, int *info);

class IllCondDetector{
public:
	typedef Eigen::SparseMatrix<double> EigenSp;
	typedef Eigen::VectorXd				VX;
public:
	IllCondDetector();
	IllCondDetector(EigenSp const &K);
	~IllCondDetector();

public:
	// I\O
	void inline SetParm(int _ns, int _nl, int _condthres, int _gap);
	double GetCondNum() const { return rcond_num_; }
	
	// Library Compatibility
	void EigenLap(EigenSp const &K);

	/* Condition number of the stiffness matrix */
	double	ComputeCondNum();

	/* Root Mean Square Equilibrium Error Check */
	double  EquilibriumError(EigenSp const &K, VX const &D, VX const &F);
	
	void Debug();
private:
	int			ns_;			// ns_ : The number of smallest eigenpairs
	int			nl_;			// nl_ : The number of largest  eigenpairs
	int			cond_thres_;	// cond_thres_ : The condition number threshold for triggering analysis
	int			gap_;			// gap_ : The order of the gap between a cluster of smallest eigenvalues
								// and the next largest eigen values

	/*
	* Matrices are well-conditioned if the 
	* reciprocal condition number is near 1 and ill-conditioned if it is near zero.
	*
	* About numerical value of condition number, please refer to:
	* http://math.stackexchange.com/questions/675474/what-is-the-practical-impact-of-a-matrixs-condition-number
	* "Condition number exceeds 10e10 could be problematic. condition number from 10e^3~6 could be acceptable."
	*/
	int			N_;				// N_       : the matrix's row number
	double		rcond_num_;		// rcond_num :the reciprocal of the condition number
	double		*A_;			// A[]		: LAPACK storage of the matrix
	double		Anorm_;			// Anorm_	: 1 norm = max_j{ sum_{i}abs(a_ij)}

	bool		debug_;
};

#endif