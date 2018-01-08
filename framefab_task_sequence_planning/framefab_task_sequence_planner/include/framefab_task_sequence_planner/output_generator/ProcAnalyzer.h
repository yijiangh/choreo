/*
* ==========================================================================
*
*		class:	ProcAnalyzer
*
*		This file is part of the implementation of
*
*		<Sustainable Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
*
*		Description:	This module take charge of output result sequence and associated
*		printing angles to KUKA robot source code generation Grasshopper file.
*
*		Version:  2.0
*		Created:  Oct/10/2015
*		Updated: Aug/24/2016
*
*		Author:  Guoxian Song, Xin Hu, Yijiang Huang, 
*		Company:  GCL@USTC
* ==========================================================================
*/

#pragma once

#include "framefab_task_sequence_planner/sequence_analyzers/SeqAnalyzer.h"

using namespace std;

struct Process
{
  bool fan_state_;
  point	start_;
  point	end_;
  std::vector<GeoV3> normal_;
};

class ProcAnalyzer
{
 public:
  ProcAnalyzer();
  ProcAnalyzer(SeqAnalyzer *seqanalyzer, const char *path);
  ~ProcAnalyzer();

  bool ProcPrint();

 private:
  bool WriteJson();

  bool IfPointInVector(point p);
  bool IfCoOrientation(GeoV3 a, vector<GeoV3> &b);
  void CheckProcess(Process &a);
  void Filter(Process &a);
  void ColorMap(double cost, double &r, double &g, double &b);

  inline double truncDigits(double v, double scale) { return ((int)(v / scale)*scale); }

 private:
  SeqAnalyzer* ptr_seqanalyzer_;
  const char* path_;

  vector<point>	exist_point_;
  ExtruderCone extruder_;

  vector<Process> process_list_;

  bool debug_;
  int support_;

  double MaxEdgeAngle_;
};

