/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	MainWindow
*
*		Description: Qt main window.
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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QLayout>
#include <QPushButton>
#include <QCheckBox>
#include <QRadioButton>
#include <QToolButton>
#include <QGroupBox>
#include <QSpinBox>
#include <QMessageBox>
#include <QKeyEvent>
#include <QSlider>
#include <QInputDialog>
#include <QLineEdit>

#include <framefab_input_gui/renderingwidget.h>
#include <ui_framefab_mainwindow.h>

enum EdgeRenderMode
{
	NONE,
	EDGE,
	HEAT,
	ORDER,
};

namespace Ui
{
class FrameFabMainWindow;
}

class RenderingWidget;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
	MainWindow(QWidget *parent = 0);
	~MainWindow();

private:
	void	CreateActions();
	void	CreateMenus();
	void	CreateLabels();
	void	CreateSpinBoxes();
	void	CreateLineEdits();
	void	CreateCheckBoxes();
	void	CreateRadioButtons();
	void	CreatePushButtons();
	void	CreateToolButtons();
	void	CreateSliders();
	void	CreateGroups();
	void	CreateDialogs();

protected:
	void	keyPressEvent(QKeyEvent *e);
	void	keyReleaseEvent(QKeyEvent *e);

Q_SIGNALS:
	void	ChangeEdgeMode(int);
	void	SendFrameFabParas(
		double, double, double, 
		bool, bool
		);
	void	SendFrameFabCutParas(
		double, double, double, 
		bool, bool
		);
	void	SendOneLayerSearchParas(double, double, double);
	void	SendDeformParas(double, double, double);
	void	SendProjectionParas(double);
	void	SendSaveOBJParas(QString);
	void	SendSavePWFParas(
				bool, bool,  
				bool, bool, 
				bool, int, int, 
				QString
			);
	void	SendExportParas(
				int, int, 
				QString, QString, QString
			);

public Q_SLOTS:
	void	OpenFile();

	void	ChooseBaseClicked(bool down);
	void	ChooseCeilingClicked(bool down);
	void	ChooseSubGClicked(bool down);

	/* mode = 1: normal fiber rountine; mode = 0: deformation calculation*/
	void	GetFrameFabParas();
	void	GetFrameFabCutParas();
	void	GetOneLayerSearchParas();
	void	GetDeformParas();
	void	GetProjectionParas();
	void	GetSaveParas();
	void	GetExportParas();
	void	GetPath();

	void	CheckEdgeMode();
	void	SwitchParaBox();

	void	SetOrderSlider(int value);
	void	SetMaxOrderSlider(int max_value);

	void	SetMinLayer(int min_value);
	void	SetMaxLayer(int max_value);

	void	OpenSaveDialog();
	void	OpenExportDialog();

	void	ShowMeshInfo(int npoint, int nedge);
	void	ShowCapturedVert(int id, int degree);
	void	ShowCapturedEdge(int id, double len);
	void	ShowScale(double scale);
	void	ShowLayerInfo(int layer_id, int total_id);

	void	ShowAbout();
	void	ShowError(QString error_msg);

	void	Reset();

 protected:
	Ui::FrameFabMainWindow ui;

 private:
	// Basic
	QMenu				*menu_file_;
	QMenu				*menu_display_;
	//QMenu				*menu_help_;
	QMenu				*menu_debug_;

	QAction				*action_new_;
	QAction				*action_open_;
	QAction				*action_save_;
	QAction				*action_import_;
	QAction				*action_export_;
	//QAction				*action_exportrender_;

	QAction				*action_background_;

	//QAction				*action_about_;

	QAction				*action_terminal_;
	QAction				*action_file_;

	// Labels
	QLabel				*label_meshinfo_;
	QLabel				*label_operatorinfo_;
	QLabel				*label_modeinfo_;
	QLabel				*label_capture_;
	QLabel				*label_layer_;

	QLabel				*label_wp_;
	QLabel				*label_wa_;
	QLabel				*label_wi_;

	QLabel				*label_scale_;
	QLabel				*label_prolen_;

	QLabel				*label_from1_;
	QLabel				*label_to1_;
	QLabel				*label_from2_;
	QLabel				*label_to2_;

	// Spinboxes
	QDoubleSpinBox		*spinbox_wp_;
	QDoubleSpinBox		*spinbox_wa_;
	QDoubleSpinBox		*spinbox_wi_;

	QDoubleSpinBox		*spinbox_scale_;
	QDoubleSpinBox		*spinbox_prolen_;

	QSpinBox			*spinbox_minlayer1_;
	QSpinBox			*spinbox_maxlayer1_;
	QSpinBox			*spinbox_minlayer2_;
	QSpinBox			*spinbox_maxlayer2_;

	// Lineedits
	QLineEdit			*lineedit_vertpath_;
	QLineEdit			*lineedit_linepath_;
	QLineEdit			*lineedit_renderpath_;
	QLineEdit			*lineedit_pwfpath_;

	// Checkboxes
	QCheckBox			*checkbox_point_;
	QCheckBox			*checkbox_light_;
	QCheckBox			*checkbox_axes_;
	QCheckBox			*checkbox_savevert_;
	QCheckBox			*checkbox_saveline_;
	QCheckBox			*checkbox_savepillar_;
	QCheckBox			*checkbox_saveceiling_;
	QCheckBox			*checkbox_savecut_;

	// Radiobuttons
	QRadioButton		*radiobutton_heat_;
	QRadioButton		*radiobutton_order_;
	QRadioButton		*radiobutton_none_;

	// Pushbuttons
	QPushButton			*pushbutton_rotatexy_;
	QPushButton			*pushbutton_rotatexz_;
	QPushButton			*pushbutton_rotateyz_;
	QPushButton			*pushbutton_lastedge_;
	QPushButton			*pushbutton_nextedge_;
	QPushButton			*pushbutton_lastlayer_;
	QPushButton			*pushbutton_nextlayer_;
	QPushButton			*pushbutton_framefabprint_;
	QPushButton			*pushbutton_framefabcut_;
	//QPushButton			*pushbutton_deformation_;
	QPushButton			*pushbutton_project_;
	//QPushButton			*pushbutton_rightarrow_;
	//QPushButton			*pushbutton_leftarrow_;
	QPushButton			*pushbutton_save_;
	QPushButton			*pushbutton_export_;
	QPushButton			*pushbutton_exportvert_;
	QPushButton			*pushbutton_exportline_;
	QPushButton			*pushbutton_exportpath_;

	// Toolbuttons
	QToolButton			*toolbutton_choosebase_;
	QToolButton			*toolbutton_chooseb_ceiling_;
	//QToolButton			*toolbutton_choosesubg_;

	// Sliders
	QSlider				*slider_order_;

	// Groupboxes
	QGroupBox			*groupbox_render_;
	QGroupBox			*groupbox_edge_;
	QGroupBox			*groupbox_orderdisplay_;
	QGroupBox			*groupbox_edit_;
	QGroupBox			*groupbox_fiber_;
	QGroupBox			*groupbox_meshpara_;
	QGroupBox			*groupbox_seqpara_;
	//QGroupBox			*groupbox_debug_;
	QGroupBox			*groupbox_sep1_;
	QGroupBox			*groupbox_sep2_;
	QGroupBox			*groupbox_exportvert_;
	QGroupBox			*groupbox_exportline_;
	QGroupBox			*groupbox_exportlayer_;
	QGroupBox			*groupbox_exportpath_;
	QGroupBox			*groupbox_saveinfo_;
	QGroupBox			*groupbox_savelayer_;

	// Dialogs
	QDialog				*dialog_save_;
	QDialog				*dialog_export_;

	EdgeRenderMode		edge_render_;

	RenderingWidget		*renderingwidget_;
};

#endif // MAINWINDOW_H
