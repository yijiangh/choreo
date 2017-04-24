/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	Statistics
*
*		Description: Statistic data output
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

#ifndef STATISTICS_H
#define STATISTICS_H

#include <iostream>
#include <fstream>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <vector>
#include <string>

using namespace std;
using namespace Eigen;

#define PATH "$HOME"

class Statistics{
public:
	Statistics(){}
	Statistics(std::string _name, Eigen::VectorXd _VX, int _iteration, char *ptr_path = PATH) 
		:name_(_name),iteration_(_iteration)
	{
		Vx_ = _VX;
		store_path_ = string(ptr_path);
	}
	Statistics(std::string _name, Eigen::VectorXd _VX, char *ptr_path = PATH)
		:name_(_name),iteration_(-1)
	{
		Vx_ = _VX;
		store_path_ = string(ptr_path);
	}

	Statistics(std::string _name, Eigen::SparseMatrix<double> SpMat, char *ptr_path = PATH)
		:name_(_name), iteration_(-1)
	{ 
		SpMat_ = SpMat; 
		store_path_ = string(ptr_path);
	}
	
	Statistics(std::string _name, Eigen::MatrixXd Mat, char *ptr_path = PATH)
		: name_(_name)
	{
		denMat_ = Mat; 
		store_path_ = string(ptr_path);
	}

	Statistics(std::string _name, std::vector<double> _stdvec, char *ptr_path = PATH)
		: name_(_name), iteration_(-1)
	{
		stdVec_ = _stdvec;
		store_path_ = string(ptr_path);
	}
	~Statistics(){}

	void StreamVectorOutPut();
	void StreamSpMatrixOutput();
	void StreamDenMatrixOutput();;

	void GenerateVectorFile();
	void GenerateMatrixFile();
	void GenerateSpFile();
	void GenerateStdVecFile();
	bool UniqueFileName();

	std::string						name_;
	int								iteration_;
	std::string						lastDir_;
	std::string						filename_;
	std::string						store_path_;
	Eigen::SparseMatrix<double>		SpMat_;
	Eigen::MatrixXd					denMat_;
	Eigen::VectorXd					Vx_;
	std::vector<double>				stdVec_;
};

#endif // STATISTICS_H
