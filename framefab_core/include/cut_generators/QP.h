/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	QP
*
*		Description:
*
*		Version:  2.0
*		Created:  Oct/20/2015
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

#ifndef ptr_qp_H
#define ptr_qp_H

#include <Eigen/Sparse>
#include <ostream>

#define MYNAN std::numeric_limits<double>::quiet_NaN()
#define MYINF std::numeric_limits<double>::infinity()

	/** Solves the quadratic programming problem:
	min 0.5* xt*H*x + ft*x subject to A*x <= b, C*x = d, x >= lb, x <= ub
	*/

	class Cone{
		// A scaled cone; s * x(idxs(0)) >= sqrt( sum_{0<i<idxs.size()} x(idxs(i))^2)
	public:
		Cone(){ s = 1.; }
		Cone(const std::vector<int>& _i, double _s = 1.){ idxs = _i; s = _s; }

		std::vector<int> idxs;
		double s;
	};

	class QP{
	public:
		typedef Eigen::SparseMatrix<double> S;
		typedef Eigen::VectorXd V;
		typedef std::vector<Cone> Cones;

		// A scaled cone with varibles x_1, ..., x_n and scale s represents quadratic cone constraint
		// x_1 * s >= \sqrt{ x_2^2 + ... = x_n^2}
		class ScaledCone{
		public:
			ScaledCone(const Cone &cone_, double scale_ = 1.0) :
				cone(cone_), scale(scale_){}

			Cone cone;
			double scale;

		};

		typedef std::vector<ScaledCone> ScaledCones;

		QP():storeVariables_(false), storePath_(std::string("./")){ ; }
		virtual ~QP(){ ; }

		virtual bool solve(
			const S& H, const V& f,
			const S& C, const V& d,
			V& _x,
			bool _debug = false) = 0;

		virtual bool solve(
			const S& H, const V& f, 
			V &_x,
			const V &_y,
			const double& d_tol,
			bool _debug) = 0;

		virtual std::string report() const = 0;
		virtual double functionValue() const = 0;
		virtual int exitFlag() const = 0;

		void setStoreVariables(bool b){ storeVariables_ = b; }
		bool storeVariables() const { return storeVariables_; }
		bool setStorePath(const std::string& sp = std::string()){ storePath_ = sp; }

		friend std::ostream& operator<<(std::ostream& out, const QP& qp);

	protected:
		bool storeVariables_;		// option, store variable or not.
		std::string storePath_;
	};


#endif // ptr_qp_H
