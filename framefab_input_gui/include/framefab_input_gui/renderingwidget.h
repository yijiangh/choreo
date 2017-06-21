/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	RenderWidget
*
*		Description: Qt render widget.
*
*		Version: 2.0
*		Created: Oct/10/2015
*		Updated: Aug/24/2016
*
*		Author:  Xin Hu, Yijiang Huang, Guoxian Song
*		Company:  GCL@USTC
*		Citation:	This file has to be used in a Qt project:
*			Title:			Qt
*							An user interface platform.
*			Code Version:	msvc_2013_5.5.0 32bit
*			Availability:	https://www.qt.io/
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

#ifndef RENDERINGWIDGET_H
#define RENDERINGWIDGET_H

#include <iostream>
#include <algorithm>

#include <QtWidgets/QMenu>
#include <QtWidgets/QAction>
#include <QtOpenGL/QGLWidget>
#include <QEvent>
#include <QKeyEvent>
#include <QColorDialog>
#include <QFileDialog>
#include <QTextCodec>

#include <GL/glut.h>

#include "framefab_gui/input_ui/mainwindow.h"
#include "framefab_gui/input_ui/ArcBall.h"
//#include "framefab_input_gui/globalFunctions.h"
//#include "framefab_input_gui/FiberPrintPlugIn.h"
#include "framefab_gui/input_ui/WireFrame.h"

enum OperationMode
{
	NORMAL,
	CHOOSEBASE,
	CHOOSECEILING,
	CHOOSESUBG,
};


class MainWindow;
class CArcBall;
class Mesh3D;

class RenderingWidget : public QGLWidget
{
	Q_OBJECT

public:
	RenderingWidget(QWidget *parent, MainWindow* mainwindow = 0);
	~RenderingWidget();

public:
	void	InitDrawData();
	void	InitCapturedData();
	void	InitFiberData();
	void	InitInfoData(
				int vert_size, int edge_size,
				QString oper_info,
				QString mode_info,
				int max_slider,
				int layer_id, int total_id
			);

protected:
	void	initializeGL();
	void	resizeGL(int w, int h);
	void	paintGL();
	void	timerEvent(QTimerEvent *e);

	// mouse events
	void	mousePressEvent(QMouseEvent *e);
	void	mouseMoveEvent(QMouseEvent *e);
	void	mouseReleaseEvent(QMouseEvent *e);
	void	mouseDoubleClickEvent(QMouseEvent *e);
	void	wheelEvent(QWheelEvent *e);

	void	keyPressEvent(QKeyEvent *e);
	void	keyReleaseEvent(QKeyEvent *e);

private:
	void	Render();
	void	SetLight();

	void	DrawAxes(bool bv);
	void	DrawPoints(bool bv);
	void	DrawEdge(bool bv);
	void	DrawHeat(bool bv);
	void	DrawOrder(bool bv);

	void	CoordinatesTransform(QPoint p, double *x, double *y, double *z);
	bool	CaptureVertex(QPoint mouse);
	bool	CaptureEdge(QPoint mouse);

public Q_SLOTS:
	void	SetBackground();

	void	CheckDrawPoint(bool bv);
	void	CheckEdgeMode(int type);
	void	CheckLight(bool bv);
	void	CheckDrawAxes(bool bv);

	void	SwitchToNormal();
	void	SwitchToChooseBase();
	void	SwitchToChooseCeiling();
	void	SwitchToChooseSubG();

public Q_SLOTS:
	void	ReadFrame();
	void	WriteFrame(QString filename);
	void	WriteFrame(
				bool bVert, bool bLine, 
				bool bPillar, bool bCeiling,
				bool bCut, int min_layer, int max_layer, 
				QString filename
			);
	void	Import();
	void	Export();
	void	Export(
				int min_layer, int max_layer, 
				QString vert_path, QString line_path, QString render_path
			);
	void	ScaleFrame(double scale);

	void	FrameFabAnalysis(double Wl, double Wp, double Wa, bool terminal_output, bool file_output);
	void	CutAnalysis(double Wl, double Wp, double Wa, bool terminal_output, bool file_output);
	void	OneLayerAnalysis(double Wl, double Wp, double Wa);
	void	DeformationAnalysis(double Wl, double Wp, double Wa);

	void	ProjectBound(double len);
	void	ModifyProjection(double len);

	void	RotateXY();
	void	RotateXZ();
	void	RotateYZ();

	void	PrintOrder(int order);
	void	PrintLastStep();
	void	PrintNextStep();
	void	PrintLastLayer();
	void	PrintNextLayer();

Q_SIGNALS:
	void	ChooseBasePressed(bool);
	void	ChooseCeilingPressed(bool);
	void	ChooseSubGPressed(bool);

	void	SetOrderSlider(int);
	void	SetMaxOrderSlider(int);

	void	SetMaxLayer(int);

	void	meshInfo(int, int);
	void	operatorInfo(QString);
	void	modeInfo(QString);
	void	CapturedVert(int, int);
	void	CapturedEdge(int, double);
	void	layerInfo(int, int);

	void	Error(QString);

	void	Reset();

public:
	MainWindow		*ptr_mainwindow_;
	CArcBall		*ptr_arcball_;
	WireFrame		*ptr_frame_;

private:
	QString			last_file_dir_;
	QString			last_result_dir_;

	// eye
	GLfloat			eye_distance_;
	point			eye_goal_;
	vec				eye_direction_;
	QPoint			current_position_;

	// Render information
	OperationMode	op_mode_;
	bool			is_draw_point_;
	bool			is_draw_edge_;
	bool			is_draw_heat_;
	bool			is_draw_order_;
	bool			has_lighting_;
	bool			is_draw_axes_;

	// Fiber
//	FiberPrintPlugIn	*ptr_fiberprint_;

	vector<WF_vert*>	captured_verts_;
	vector<bool>			is_captured_vert_;
	vector<WF_edge*>	captured_edges_;
	vector<bool>			is_captured_edge_;

	float					scale_;
	int					print_order_;
};

#endif // RENDERINGWIDGET_H
