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

#ifndef FIBERPRINTPLUGIN_H
#define FIBERPRINTPLUGIN_H

// framefab utils
#include "framefab_task_sequence_planner/utils/WireFrame.h"
#include "framefab_task_sequence_planner/utils/DualGraph.h"
#include "framefab_task_sequence_planner/utils/QuadricCollision.h"
#include "framefab_task_sequence_planner/utils/Stiffness.h"

#include "framefab_task_sequence_planner/sequence_analyzers/FFAnalyzer.h"
#include "framefab_task_sequence_planner/sequence_analyzers/GecodeAnalyzer.h"

#include "framefab_task_sequence_planner/output_generator/ProcAnalyzer.h"

// ros srv
#include <framefab_msgs/TaskSequencePlanning.h>

// ros msg
#include <framefab_msgs/ElementCandidatePoses.h>
#include <framefab_msgs/WireFrameCollisionObject.h>

// robot model
#include <descartes_core/robot_model.h>
#include <pluginlib/class_loader.h>

struct WFEdgeCollisionObject
{

};

class FiberPrintPlugIn
{
 public:
  typedef Eigen::MatrixXd MX;
  typedef Eigen::VectorXd VX;

 public:
  FiberPrintPlugIn(const std::string& world_frame,
                   const std::string& hotend_group, const std::string& hotend_tcp, const std::string& hotend_base,
                   const std::string& robot_model_plugin);

  ~FiberPrintPlugIn();

 public:
  bool Init();

  bool DirectSearch();
  bool ConstructCollisionObjects(const std::vector<int>& print_queue_edge_ids,
                                 std::vector<framefab_msgs::WireFrameCollisionObject>& collision_objs);

  /* apply stiffness computation directly to the input frame shape */
  void GetDeformation();

  /* ros service */
  bool handleTaskSequencePlanning(
      framefab_msgs::TaskSequencePlanning::Request& req,
      framefab_msgs::TaskSequencePlanning::Response& res);

 public:
  WireFrame *ptr_frame_;
  DualGraph *ptr_dualgraph_;
  QuadricCollision *ptr_collision_;
  Stiffness *ptr_stiffness_;
  std::vector<framefab_msgs::ElementCandidatePoses> frame_msgs_;

  SeqAnalyzer *ptr_seqanalyzer_;
  ProcAnalyzer *ptr_procanalyzer_;

  // output saving path
  char *ptr_path_;
  FiberPrintPARM *ptr_parm_;

 private:
  Timer fiber_print_;

  bool terminal_output_;
  bool file_output_;

 private:
  descartes_core::RobotModelPtr hotend_model_;
  moveit::core::RobotModelConstPtr moveit_model_;
  pluginlib::ClassLoader<descartes_core::RobotModel> plugin_loader_; // kept around so code doesn't get unloaded
  std::string hotend_group_name_;

  std::string world_frame_;
};

#endif // FIBERPRINTPLUGIN_H