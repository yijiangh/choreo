/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	StiffnessIO
*
*		Description: This module takes charge of outputting stiffness matrix related results.
*
*		Version:  1.0
*		Created:  Mar/23/2016
*		Updated:  Aug/24/2016
*
*		Author:  Xin Hu, Yijiang Huang, Guoxian Song
*		Company:  GCL@USTC
*		Citation:	Some part of this module is modified from frame3dd_io.c
*					output related submodule.
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

#ifndef STIFFNESS_IO_H
#define STIFFNESS_IO_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <Eigen/Dense>

#include "framefab_task_sequence_planner/utils/CoordTrans.h"
#include "framefab_task_sequence_planner/utils/Statistics.h"

#include "framefab_task_sequence_planner/utils/GCommon.h"
#include "framefab_task_sequence_planner/utils/StiffnessSolver.h"

#include "framefab_task_sequence_planner/FiberPrintPARM.h"
#include "framefab_task_sequence_planner/utils/DualGraph.h"
#include "framefab_task_sequence_planner/utils/CoordTrans.h"

#define MYOUT std::cout
#define MYEND std::endl

class StiffnessIO
{
 public:
  typedef Eigen::MatrixXd MX;
  typedef Eigen::Matrix3d M3;
  typedef Eigen::VectorXd VX;
  typedef Eigen::Vector3d V3;
  typedef Eigen::VectorXi VXi;
  typedef	Eigen::MatrixXi MXi;

  typedef std::vector<V3> vec3;

 public:
  StiffnessIO(){};
  ~StiffnessIO(){};

 public:

  void	OutputPath(const char *fname, char fullpath[], const int len, char *default_outdir, int verbose);

  /*--- GnuPlot file output ---*/
  /*
  * GnuPltStaticMesh - create mesh data of deformed and undeformed mesh, use gnuplot	 Mar/20/2016
  */
  void GnuPltStaticMesh(
		  const char *fpath,
		  const char *meshpath, const char *plotpath,
		  VX &D,
		  double exagg_static, float scale,
		  DualGraph *ptr_dualgraph, WireFrame *ptr_frame
  );

  /*
  * GnuPltCubicBentBeam -											Mar/20/2016
  *	computes cubic deflection functions from end deflections
  *	and end rotations.  Saves deflected shapes to a file.
  *	These bent shapes are exact for mode-shapes, and for frames
  *	loaded at their nodes.
  */
  void GnuPltCubicBentBeam(
		  vector<point> &beam,
		  VX &D,			/* calculated deformation */
		  int dual_i, DualGraph *ptr_dualgraph, WireFrame *ptr_frame,
		  double exagg
  );

  /*
  * WriteInputData - write input data to a .3dd file			Mar/20/2016
  */
  void WriteInputData(
		  const char *fpath,
		  DualGraph *ptr_dualgraph,
		  FiberPrintPARM *ptr_parm,
		  int verbose
  );

  /*
  * SaveUpperMatrix - save a symmetric matrix of dimension [1..n][1..n]	Nov/26/2015
  * to the named file, use only upper-triangular part
  */
  void SaveUpperMatrix(char filename[], const MX &A, int n);

  /*
  * SaveDeformVector - save displacement vector of dimemsion [1...6*N]	Nov/26/2015
  * to the named file
  */		;
  void SaveDisplaceVector(char filename[], const VX &D, int n, DualGraph *ptr_dual_graph);

 public:
  void	dots(FILE *fp, int n)
  {
	  int i;
	  for (i = 1; i <= n; i++)	fprintf(fp, ".");
  }

  const char *TempDir()
  {
	  char *tmp;
	  tmp = getenv("TEMP");
	  if (tmp == NULL) {
		  fprintf(stderr,
				  "ERROR: Environment Variables %%TEMP%% and %%FRAME3DD_OUTDIR%% are not set.  "
						  "At least one of these variables must be set so that FrameFab knows where to "
						  "write its temporary files.  Set one of these variable, then re-run FrameFab.");
		  exit(15);
	  }
	  return tmp;
  }

 private:
  CoordTrans		trsf_;
  StiffnessSolver	solver_;	// solver_: LU decomposition for cubic bent beam computaion
};
#endif