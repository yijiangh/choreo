/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		Class:	QpMosek
*
*		Description:
*
*		Version: 2.0
*		Created: Oct/10/2015
*		Updated: Aug/24/2016
*
*		Author:  Xin Hu, Yijiang Huang, Guoxian Song
*		Company:  GCL@USTC
*		Citation:	This file completely rely on
*			Title:			Mosek
*							High performance software for large-scale LP, QP, SOCP, SDP
*							and MIP including interfaces to C, Java, MATLAB, .NET, R and Python. 
*			Code Version:	7.1
*			Availability:	https://www.mosek.com/
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

#ifndef QPMOSEK_H
#define QPMOSEK_H

#include <cut_generators/QP.h>
#include <utils/Timer.h>

class QPMosek : public QP
{
public:
	QPMosek();
	virtual ~QPMosek();

	/* for FrameFab graphcut: CalculateX*/
	virtual bool solve(
		const S& H, const V& f,
		const S& C, const V& d,
		V& _x,
		bool _debug = false);
	
	/* for FrameFab graphcut: CalculateD*/
	virtual bool solve(
		const S& H, const V& f, 
		V &_d,
		const V& _x,
		const double& d_tol,
		bool _debug);

	virtual std::string		report() const;
	virtual double			functionValue() const { return fVal_; }
	virtual int				exitFlag() const { return xFlag_; }

	void	setNTasks(int n){ nTasks_ = n; };
	int		nTasks() const { return nTasks_; }

	void	setThreshold(double t){ mP_ = t; }
	double	threshold(){ return mP_; }

	std::string exitFlagToString(int _xFlag) const;

private:
	Timer tSetup, tSolve;
	void*	env_;		// environment in Mosek
	double	fVal_;		// objective fdunction value
	int		xFlag_;		// solution status
	int		nTasks_;	// for multi task number
	double	mP_;		// threshold
};
#endif // QPMOSEK_H