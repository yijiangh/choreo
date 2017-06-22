#include <framefab_gui/input_ui/mainwindow.h>


MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
//	ui.mainToolBar->setVisible(false);
	this->setWindowTitle("FrameFab");
	setGeometry(200, 150, 1000, 700);
	
	renderingwidget_ = new RenderingWidget(this);
	connect(renderingwidget_, SIGNAL(Error(QString)), this, SLOT(ShowError(QString)));
	connect(renderingwidget_, SIGNAL(Reset()), this, SLOT(Reset()));

	CreateActions();
	CreateMenus();
	CreateLabels();
	CreateSpinBoxes();
	CreateLineEdits();
	CreateCheckBoxes();
	CreateSliders();
	CreateRadioButtons();
	CreatePushButtons();
	CreateToolButtons();
	CreateGroups();
	CreateDialogs();
	
	QVBoxLayout *layout_left = new QVBoxLayout;
	layout_left->addWidget(groupbox_render_);
	layout_left->addWidget(groupbox_edge_);
	layout_left->addWidget(groupbox_orderdisplay_);
	layout_left->addWidget(groupbox_edit_);
	layout_left->addWidget(groupbox_sep1_);
	layout_left->addStretch(1);

	QVBoxLayout *layout_right = new QVBoxLayout;
	layout_right->addWidget(groupbox_fiber_);
	layout_right->addWidget(groupbox_meshpara_);
	layout_right->addWidget(groupbox_seqpara_);
	//layout_right->addWidget(groupbox_debug_);
	layout_right->addWidget(groupbox_sep2_);
	layout_right->addStretch(1);

	QHBoxLayout *layout_main = new QHBoxLayout;
	layout_main->addLayout(layout_left);
	layout_main->addWidget(renderingwidget_);
	layout_main->setStretch(1, 1);
	layout_main->addLayout(layout_right);
	this->centralWidget()->setLayout(layout_main);
	
	Reset();
}


MainWindow::~MainWindow()
{
	delete renderingwidget_;
	renderingwidget_ = NULL;
}


void MainWindow::CreateActions()
{
	action_new_ = new QAction(QIcon(":/Resources/images/new.png"), tr("New"), this);
	action_new_->setShortcut(QKeySequence::New);
	action_new_->setStatusTip(tr("Create a new file"));

	action_open_ = new QAction(QIcon(":/MainWindow/Resources/images/open.png"), tr("Open..."), this);
	action_open_->setShortcuts(QKeySequence::Open);
	action_open_->setStatusTip(tr("Open an existing file"));
	connect(action_open_, SIGNAL(triggered()), renderingwidget_, SLOT(ReadFrame()));

	action_save_ = new QAction(QIcon(":/MainWindow/Resources/images/save.png"), tr("Save"), this);
	action_save_->setShortcuts(QKeySequence::Save);
	action_save_->setStatusTip(tr("Save the document to disk"));
	connect(action_save_, SIGNAL(triggered()), this, SLOT(OpenSaveDialog()));

	action_import_ = new QAction(tr("Import"), this);
	action_import_->setStatusTip(tr("Import from disk"));
	connect(action_import_, SIGNAL(triggered()), renderingwidget_, SLOT(Import()));

	action_export_ = new QAction(tr("Export"), this);
	action_export_->setStatusTip(tr("Export to disk"));
	connect(action_export_, SIGNAL(triggered()), renderingwidget_, SLOT(Export()));

	//action_exportrender_ = new QAction(tr("Export to render"), this);
	//action_exportrender_->setStatusTip(tr("Export to render"));
	//connect(action_exportrender_, SIGNAL(triggered()), this, SLOT(OpenExportDialog()));

	action_background_ = new QAction(tr("Change background"), this);
	connect(action_background_, SIGNAL(triggered()), renderingwidget_, SLOT(SetBackground()));

	//action_about_ = new QAction(tr("About"), this);
	//connect(action_about_, SIGNAL(triggered()), this, SLOT(ShowAbout()));

	action_terminal_ = new QAction(tr("Terminal Output"), this);
	action_terminal_->setCheckable(true);

	action_file_ = new QAction(tr("File Output"), this);
	action_file_->setCheckable(true);
}


void MainWindow::CreateMenus()
{
	menu_file_ = menuBar()->addMenu(tr("&File"));
	menu_file_->setStatusTip(tr("File menu"));
	menu_file_->addAction(action_new_);
	menu_file_->addAction(action_open_);
	menu_file_->addAction(action_save_);

	menu_file_->addSeparator();
	menu_file_->addAction(action_import_);
	menu_file_->addAction(action_export_);
	//menu_file_->addAction(action_exportrender_);

	menu_display_ = menuBar()->addMenu(tr("&Display"));
	menu_display_->setStatusTip(tr("Display settings"));
	menu_display_->addAction(action_background_);

	menu_debug_ = menuBar()->addMenu(tr("&Debug"));
	menu_debug_->setStatusTip(tr("Debug"));
	menu_debug_->addAction(action_terminal_);
	menu_debug_->addAction(action_file_);
}


void MainWindow::CreateLabels()
{
	label_meshinfo_ = new QLabel(QString("MeshInfo: p: %1 e: %2").arg(0).arg(0), this);
	label_meshinfo_->setAlignment(Qt::AlignCenter);
	label_meshinfo_->setMinimumSize(label_meshinfo_->sizeHint());

	label_operatorinfo_ = new QLabel(QString("Scale: 1.0"), this);
	label_operatorinfo_->setAlignment(Qt::AlignVCenter);
	
	label_modeinfo_ = new QLabel(this);

	label_capture_ = new QLabel(this);

	label_layer_ = new QLabel(this);

	statusBar()->addWidget(label_meshinfo_);
	connect(renderingwidget_, SIGNAL(meshInfo(int, int)), this, SLOT(ShowMeshInfo(int, int)));

	statusBar()->addWidget(label_modeinfo_);
	connect(renderingwidget_, SIGNAL(modeInfo(QString)), label_modeinfo_, SLOT(setText(QString)));

	statusBar()->addWidget(label_capture_);
	connect(renderingwidget_, SIGNAL(CapturedVert(int, int)), this, SLOT(ShowCapturedVert(int, int)));
	connect(renderingwidget_, SIGNAL(CapturedEdge(int, double)), this, SLOT(ShowCapturedEdge(int, double)));

	statusBar()->addWidget(label_layer_);
	connect(renderingwidget_, SIGNAL(layerInfo(int, int)), this, SLOT(ShowLayerInfo(int, int)));

	statusBar()->addWidget(label_operatorinfo_);
	connect(renderingwidget_, SIGNAL(operatorInfo(QString)), label_operatorinfo_, SLOT(setText(QString)));

	label_wp_		= new QLabel(QString("Seq Wp: "), this);
	label_wa_		= new QLabel(QString("Seq Wa: "), this);
	label_wi_		= new QLabel(QString("Seq Wi: "), this);

	label_scale_	= new QLabel(QString("Scale: "), this);
	label_prolen_	= new QLabel(QString("Pillar length: "), this);

	label_from1_	= new QLabel(QString("From"), this);
	label_to1_		= new QLabel(QString("To"), this);
	label_from2_	= new QLabel(QString("From"), this);
	label_to2_		= new QLabel(QString("To"), this);
}


void MainWindow::CreateSpinBoxes()
{
	spinbox_wp_ = new QDoubleSpinBox(this);
	spinbox_wp_->setFixedWidth(140);
	spinbox_wp_->setDecimals(2);
	spinbox_wp_->setRange(0, 10000);
	spinbox_wp_->setValue(1.0);
	spinbox_wp_->setSingleStep(1);

	spinbox_wa_ = new QDoubleSpinBox(this);
	spinbox_wa_->setFixedWidth(140);
	spinbox_wa_->setDecimals(2);
	spinbox_wa_->setRange(0, 10000);
	spinbox_wa_->setValue(1.0);
	spinbox_wa_->setSingleStep(1);

	spinbox_wi_ = new QDoubleSpinBox(this);
	spinbox_wi_->setFixedWidth(140);
	spinbox_wi_->setDecimals(2);
	spinbox_wi_->setRange(0, 10000);
	spinbox_wi_->setValue(5.0);
	spinbox_wi_->setSingleStep(1);

	spinbox_scale_ = new QDoubleSpinBox(this);
	spinbox_scale_->setFixedWidth(140);
	spinbox_scale_->setDecimals(1);
	spinbox_scale_->setRange(0.1, 100);
	spinbox_scale_->setValue(1.0);
	spinbox_scale_->setSingleStep(0.5);
	connect(spinbox_scale_, SIGNAL(valueChanged(double)), this, SLOT(ShowScale(double)));
	connect(spinbox_scale_, SIGNAL(valueChanged(double)), renderingwidget_, SLOT(ScaleFrame(double)));

	spinbox_prolen_ = new QDoubleSpinBox(this);
	spinbox_prolen_->setFixedWidth(140);
	spinbox_prolen_->setDecimals(1);
	spinbox_prolen_->setRange(0, 1000);
	spinbox_prolen_->setValue(10.0);
	spinbox_prolen_->setSingleStep(1);
	connect(spinbox_prolen_, SIGNAL(valueChanged(double)), renderingwidget_, SLOT(ModifyProjection(double)));

	spinbox_minlayer1_ = new QSpinBox(this);
	spinbox_minlayer1_->setFixedWidth(60);
	spinbox_minlayer1_->setRange(1, 1);
	spinbox_minlayer1_->setValue(1);
	spinbox_minlayer1_->setSingleStep(1);

	spinbox_maxlayer1_ = new QSpinBox(this);
	spinbox_maxlayer1_->setFixedWidth(60);
	spinbox_maxlayer1_->setRange(1, 1);
	spinbox_maxlayer1_->setValue(1);
	spinbox_maxlayer1_->setSingleStep(1);

	spinbox_minlayer2_ = new QSpinBox(this);
	spinbox_minlayer2_->setFixedWidth(60);
	spinbox_minlayer2_->setRange(1, 1);
	spinbox_minlayer2_->setValue(1);
	spinbox_minlayer2_->setSingleStep(1);

	spinbox_maxlayer2_ = new QSpinBox(this);
	spinbox_maxlayer2_->setFixedWidth(60);
	spinbox_maxlayer2_->setRange(1, 1);
	spinbox_maxlayer2_->setValue(1);
	spinbox_maxlayer2_->setSingleStep(1);
	connect(renderingwidget_, SIGNAL(SetMaxLayer(int)), this, SLOT(SetMaxLayer(int)));
	connect(spinbox_maxlayer1_, SIGNAL(valueChanged(int)), this, SLOT(SetMinLayer(int)));
	connect(spinbox_maxlayer2_, SIGNAL(valueChanged(int)), this, SLOT(SetMinLayer(int)));

	//pushbutton_scale_ = new QPushButton(tr("Scale"), this);
	//pushbutton_scale_->setFixedSize(80, 25);
	//connect(pushbutton_scale_, SIGNAL(clicked()), this, SLOT(CheckScale()));
	//connect(this, SIGNAL(ChangeScale(double)), renderingwidget_, SLOT(ScaleFrame(double)));
}


void MainWindow::CreateLineEdits()
{
	lineedit_vertpath_ = new QLineEdit(this);
	lineedit_linepath_ = new QLineEdit(this);
	lineedit_renderpath_ = new QLineEdit(this);

	lineedit_pwfpath_ = new QLineEdit(this);
	lineedit_pwfpath_->setVisible(false);
}


void MainWindow::CreateCheckBoxes()
{
	checkbox_point_ = new QCheckBox(tr("Point"), this);
	connect(checkbox_point_, SIGNAL(clicked(bool)), renderingwidget_, SLOT(CheckDrawPoint(bool)));
	checkbox_point_->setChecked(true);

	checkbox_light_ = new QCheckBox(tr("Light"), this);
	connect(checkbox_light_, SIGNAL(clicked(bool)), renderingwidget_, SLOT(CheckLight(bool)));

	checkbox_axes_ = new QCheckBox(tr("Axes"), this);
	connect(checkbox_axes_, SIGNAL(clicked(bool)), renderingwidget_, SLOT(CheckDrawAxes(bool)));

	checkbox_savevert_ = new QCheckBox(tr("Vert"), this);
	checkbox_saveline_ = new QCheckBox(tr("Line"), this);
	checkbox_savepillar_ = new QCheckBox(tr("Pillar"), this);
	checkbox_saveceiling_ = new QCheckBox(tr("Ceiling"), this);
	checkbox_savecut_ = new QCheckBox(tr("Cut"), this);
}


void MainWindow::CreateSliders()
{
	slider_order_ = new QSlider(Qt::Horizontal, this);
	slider_order_->setFixedSize(80, 25);
	slider_order_->setMinimum(0);
	slider_order_->setMaximum(50);
	slider_order_->setSingleStep(1);
	connect(slider_order_, SIGNAL(valueChanged(int)), renderingwidget_, SLOT(PrintOrder(int)));
	connect(renderingwidget_, SIGNAL(SetOrderSlider(int)), this, SLOT(SetOrderSlider(int)));
	connect(renderingwidget_, SIGNAL(SetMaxOrderSlider(int)), this, SLOT(SetMaxOrderSlider(int)));
}


void MainWindow::CreateRadioButtons()
{
	radiobutton_heat_ = new QRadioButton(tr("Heat"), this);
	connect(radiobutton_heat_, SIGNAL(clicked(bool)), this, SLOT(CheckEdgeMode()));

	radiobutton_order_ = new QRadioButton(tr("Order"), this);
	connect(radiobutton_order_, SIGNAL(clicked(bool)), this, SLOT(CheckEdgeMode()));

	radiobutton_none_ = new QRadioButton(tr("None"), this);
	radiobutton_none_->setVisible(false);
	radiobutton_none_->setChecked(true);
	edge_render_ = NONE;
}


void MainWindow::CreatePushButtons()
{
	pushbutton_rotatexy_ = new QPushButton(tr("RotateXY"), this);
	pushbutton_rotatexz_ = new QPushButton(tr("RotateXZ"), this);
	pushbutton_rotateyz_ = new QPushButton(tr("RotateYZ"), this);
	pushbutton_rotatexy_->setFixedSize(80, 25);
	pushbutton_rotatexz_->setFixedSize(80, 25);
	pushbutton_rotateyz_->setFixedSize(80, 25);
	connect(pushbutton_rotatexy_, SIGNAL(clicked()), renderingwidget_, SLOT(RotateXY()));
	connect(pushbutton_rotatexz_, SIGNAL(clicked()), renderingwidget_, SLOT(RotateXZ()));
	connect(pushbutton_rotateyz_, SIGNAL(clicked()), renderingwidget_, SLOT(RotateYZ()));

	pushbutton_lastedge_ = new QPushButton(tr("<"), this);
	pushbutton_lastedge_->setFixedSize(35, 20);
	connect(pushbutton_lastedge_, SIGNAL(clicked()), renderingwidget_, SLOT(PrintLastStep()));

	pushbutton_nextedge_ = new QPushButton(tr(">"), this);
	pushbutton_nextedge_->setFixedSize(35, 20);
	connect(pushbutton_nextedge_, SIGNAL(clicked()), renderingwidget_, SLOT(PrintNextStep()));

	pushbutton_lastlayer_ = new QPushButton(tr("<<"), this);
	pushbutton_lastlayer_->setFixedSize(35, 20);
	connect(pushbutton_lastlayer_, SIGNAL(clicked()), renderingwidget_, SLOT(PrintLastLayer()));

	pushbutton_nextlayer_ = new QPushButton(tr(">>"), this);
	pushbutton_nextlayer_->setFixedSize(35, 20);
	connect(pushbutton_nextlayer_, SIGNAL(clicked()), renderingwidget_, SLOT(PrintNextLayer()));

	pushbutton_framefabprint_ = new QPushButton(tr("FrameFab Print"), this);
	pushbutton_framefabprint_->setFixedSize(140, 35);
	connect(pushbutton_framefabprint_, SIGNAL(clicked()), this, SLOT(GetFrameFabParas()));
	connect(this, 
		SIGNAL(SendFrameFabParas(double, double, double, bool, bool)),
		renderingwidget_, 
		SLOT(FrameFabAnalysis(double, double, double, bool, bool)));

	pushbutton_framefabcut_ = new QPushButton(tr("FrameFab Cut"), this);
	pushbutton_framefabcut_->setFixedSize(140, 35);
	connect(pushbutton_framefabcut_, SIGNAL(clicked()), this, SLOT(GetFrameFabCutParas()));
	connect(this,
		SIGNAL(SendFrameFabCutParas(double, double, double, bool, bool)),
		renderingwidget_,
		SLOT(CutAnalysis(double, double, double, bool, bool)));

	//pushbutton_deformation_ = new QPushButton(tr("Deformation"), this);
	//pushbutton_deformation_->setFixedSize(140, 35);
	//connect(pushbutton_deformation_, SIGNAL(clicked()), this, SLOT(GetDeformParas()));
	//connect(this,
	//	SIGNAL(SendDeformParas(double, double, double)),
	//	renderingwidget_,
	//	SLOT(DeformationAnalysis(double, double, double)));

	pushbutton_project_ = new QPushButton(tr("Project"), this);
	pushbutton_project_->setFixedSize(140, 35);
	connect(pushbutton_project_, SIGNAL(clicked()), this, SLOT(GetProjectionParas()));
	connect(this, SIGNAL(SendProjectionParas(double)), renderingwidget_, SLOT(ProjectBound(double)));
	
	//pushbutton_rightarrow_ = new QPushButton(tr(">>"), this);
	//pushbutton_rightarrow_->setFlat(true);
	//pushbutton_rightarrow_->setFixedSize(20, 20);
	//connect(pushbutton_rightarrow_, SIGNAL(clicked()), this, SLOT(SwitchParaBox()));

	//pushbutton_leftarrow_ = new QPushButton(tr("<<"), this);
	//pushbutton_leftarrow_->setFlat(true);
	//pushbutton_leftarrow_->setFixedSize(20, 20);
	//connect(pushbutton_leftarrow_, SIGNAL(clicked()), this, SLOT(SwitchParaBox()));

	pushbutton_save_ = new QPushButton(tr("Save"), this);
	connect(pushbutton_save_, SIGNAL(clicked()), this, SLOT(GetSaveParas()));
	connect(this, SIGNAL(SendSaveOBJParas(QString)), 
		renderingwidget_, SLOT(WriteFrame(QString)));
	connect(this, SIGNAL(SendSavePWFParas(bool, bool, bool, bool, bool, int, int, QString)),
		renderingwidget_, SLOT(WriteFrame(bool, bool, bool, bool, bool, int, int, QString)));

	pushbutton_export_ = new QPushButton(tr("Export"), this);
	connect(pushbutton_export_, SIGNAL(clicked()), this, SLOT(GetExportParas()));
	connect(this, SIGNAL(SendExportParas(int, int, QString, QString, QString)),
		renderingwidget_, SLOT(Export(int, int, QString, QString, QString)));

	pushbutton_exportvert_ = new QPushButton(tr("..."), this);
	pushbutton_exportvert_->setFixedWidth(30);
	connect(pushbutton_exportvert_, SIGNAL(clicked()), this, SLOT(GetPath()));

	pushbutton_exportline_ = new QPushButton(tr("..."), this);
	pushbutton_exportline_->setFixedWidth(30);
	connect(pushbutton_exportline_, SIGNAL(clicked()), this, SLOT(GetPath()));

	pushbutton_exportpath_ = new QPushButton(tr("..."), this);
	pushbutton_exportpath_->setFixedWidth(30);
	connect(pushbutton_exportpath_, SIGNAL(clicked()), this, SLOT(GetPath()));
}


void MainWindow::CreateToolButtons()
{
	toolbutton_choosebase_ = new QToolButton(this);
	toolbutton_choosebase_->setText(tr("Choose base"));
	toolbutton_choosebase_->setFixedSize(140, 35);
	connect(toolbutton_choosebase_, SIGNAL(clicked()), renderingwidget_, SLOT(SwitchToChooseBase()));

	toolbutton_chooseb_ceiling_ = new QToolButton(this);
	toolbutton_chooseb_ceiling_->setText(tr("Choose ceiling"));
	toolbutton_chooseb_ceiling_->setFixedSize(140, 35);
	connect(toolbutton_chooseb_ceiling_, SIGNAL(clicked()), renderingwidget_, SLOT(SwitchToChooseCeiling()));

	//toolbutton_choosesubg_ = new QToolButton(this);
	//toolbutton_choosesubg_->setText(tr("Choose\nsubgraph"));
	//toolbutton_choosesubg_->setFixedSize(80, 50);
	//connect(toolbutton_choosesubg_, SIGNAL(clicked()), renderingwidget_, SLOT(SwitchToChooseSubG()));

	connect(renderingwidget_, SIGNAL(ChooseBasePressed(bool)), this, SLOT(ChooseBaseClicked(bool)));
	connect(renderingwidget_, SIGNAL(ChooseCeilingPressed(bool)), this, SLOT(ChooseCeilingClicked(bool)));
	connect(renderingwidget_, SIGNAL(ChooseSubGPressed(bool)), this, SLOT(ChooseSubGClicked(bool)));
}


void MainWindow::CreateGroups()
{
	// render group
	connect(this, SIGNAL(ChangeEdgeMode(int)), renderingwidget_, SLOT(CheckEdgeMode(int)));

	groupbox_render_ = new QGroupBox(tr("Render"), this);
	groupbox_render_->setFlat(true);
	groupbox_render_->setCheckable(true);
	QVBoxLayout* render_layout = new QVBoxLayout(groupbox_render_);
	render_layout->addWidget(checkbox_point_);
	render_layout->addWidget(checkbox_light_);
	render_layout->addWidget(checkbox_axes_);

	// edge group
	groupbox_edge_ = new QGroupBox(tr("Edge"), this);
	groupbox_edge_->setCheckable(true);
	connect(groupbox_edge_, SIGNAL(clicked(bool)), this, SLOT(CheckEdgeMode()));

	QVBoxLayout* edge_layout = new QVBoxLayout(groupbox_edge_);
	edge_layout->addWidget(radiobutton_heat_);
	edge_layout->addWidget(radiobutton_order_);
	edge_layout->addWidget(radiobutton_none_);

	// order display group
	groupbox_orderdisplay_ = new QGroupBox(tr("Display"), this);
	groupbox_orderdisplay_->setFlat(true);

	QVBoxLayout* orderbar_layout = new QVBoxLayout();
	orderbar_layout->addWidget(slider_order_);

	QGridLayout* orderbutton_layout = new QGridLayout();
	orderbutton_layout->addWidget(pushbutton_lastedge_, 0, 0);
	orderbutton_layout->addWidget(pushbutton_nextedge_, 0, 1);
	orderbutton_layout->addWidget(pushbutton_lastlayer_, 1, 0);
	orderbutton_layout->addWidget(pushbutton_nextlayer_, 1, 1);

	QVBoxLayout* orderdisplay_layout = new QVBoxLayout(groupbox_orderdisplay_);
	orderdisplay_layout->addLayout(orderbar_layout);
	orderdisplay_layout->addLayout(orderbutton_layout);

	// edit group
	groupbox_edit_ = new QGroupBox(tr("Edit"), this);
	groupbox_edit_->setFlat(true);
	QVBoxLayout* edit_layout = new QVBoxLayout(groupbox_edit_);
	edit_layout->addWidget(pushbutton_rotatexy_);
	edit_layout->addWidget(pushbutton_rotatexz_);
	edit_layout->addWidget(pushbutton_rotateyz_);
	//edit_layout->addWidget(toolbutton_choosesubg_);

	// separator group
	groupbox_sep1_ = new QGroupBox(this);
	groupbox_sep1_->setFlat(true);

	// fiber group
	groupbox_fiber_ = new QGroupBox(tr("Fiber"), this);
	groupbox_fiber_->setFlat(true);
	QVBoxLayout *fiber_layout = new QVBoxLayout(groupbox_fiber_);
	fiber_layout->addWidget(pushbutton_framefabprint_);
	fiber_layout->addWidget(pushbutton_framefabcut_);
	//fiber_layout->addWidget(pushbutton_deformation_);
	fiber_layout->addWidget(toolbutton_choosebase_);
	fiber_layout->addWidget(toolbutton_chooseb_ceiling_);
	fiber_layout->addWidget(pushbutton_project_);

	groupbox_meshpara_ = new QGroupBox(tr("Mesh parameter"), this);
	groupbox_meshpara_->setFlat(true);
	QVBoxLayout *meshpara_layout = new QVBoxLayout(groupbox_meshpara_);
	meshpara_layout->addWidget(label_scale_);
	meshpara_layout->addWidget(spinbox_scale_);
	meshpara_layout->addWidget(label_prolen_);
	meshpara_layout->addWidget(spinbox_prolen_);

	// seqpara group
	groupbox_seqpara_ = new QGroupBox(tr("Seq parameter"), this);
	groupbox_seqpara_->setFlat(true);
	QVBoxLayout *seqpara_layout = new QVBoxLayout(groupbox_seqpara_);
	seqpara_layout->addWidget(label_wp_);
	seqpara_layout->addWidget(spinbox_wp_);
	seqpara_layout->addWidget(label_wa_);
	seqpara_layout->addWidget(spinbox_wa_);
	seqpara_layout->addWidget(label_wi_);
	seqpara_layout->addWidget(spinbox_wi_);
	//seqpara_layout->addWidget(pushbutton_rightarrow_);

	// debug group
	//groupbox_debug_ = new QGroupBox(tr("Debug"), this);
	//groupbox_debug_->setFlat(true);
	//QVBoxLayout *debug_layout = new QVBoxLayout(groupbox_debug_);
	//debug_layout->addWidget(pushbutton_leftarrow_);

	// separator group
	groupbox_sep2_ = new QGroupBox(this);
	groupbox_sep2_->setFlat(true);


	// export group
	groupbox_exportvert_ = new QGroupBox(tr("Export vert"), this);
	groupbox_exportvert_->setCheckable(true);
	QHBoxLayout *exportvert_layout = new QHBoxLayout(groupbox_exportvert_);
	exportvert_layout->addWidget(lineedit_vertpath_);
	exportvert_layout->addWidget(pushbutton_exportvert_);

	groupbox_exportline_ = new QGroupBox(tr("Export line"), this);
	groupbox_exportline_->setCheckable(true);
	QHBoxLayout *exportline_layout = new QHBoxLayout(groupbox_exportline_);
	exportline_layout->addWidget(lineedit_linepath_);
	exportline_layout->addWidget(pushbutton_exportline_);

	groupbox_exportpath_ = new QGroupBox(tr("Export rendering path"), this);
	groupbox_exportpath_->setCheckable(true);
	QHBoxLayout *exportpath_layout = new QHBoxLayout(groupbox_exportpath_);
	exportpath_layout->addWidget(lineedit_renderpath_);
	exportpath_layout->addWidget(pushbutton_exportpath_);

	groupbox_exportlayer_ = new QGroupBox(tr("Export layer"), this);
	QHBoxLayout *exportlayer_layout = new QHBoxLayout(groupbox_exportlayer_);
	exportlayer_layout->addWidget(label_from2_);
	exportlayer_layout->addWidget(spinbox_minlayer2_);
	exportlayer_layout->addWidget(label_to2_);
	exportlayer_layout->addWidget(spinbox_maxlayer2_);

	// save
	groupbox_saveinfo_ = new QGroupBox(tr("Options"), this);
	QGridLayout *saveinfo_layout = new QGridLayout(groupbox_saveinfo_);
	saveinfo_layout->addWidget(checkbox_savevert_, 1, 1);
	saveinfo_layout->addWidget(checkbox_saveline_, 1, 2);
	saveinfo_layout->addWidget(checkbox_savepillar_, 2, 1);
	saveinfo_layout->addWidget(checkbox_saveceiling_, 2, 2);
	saveinfo_layout->addWidget(checkbox_savecut_, 3, 1);

	groupbox_savelayer_ = new QGroupBox(tr("Save layer"), this);
	QHBoxLayout *savelayer_layout = new QHBoxLayout(groupbox_savelayer_);
	savelayer_layout->addWidget(label_from1_);
	savelayer_layout->addWidget(spinbox_minlayer1_);
	savelayer_layout->addWidget(label_to1_);
	savelayer_layout->addWidget(spinbox_maxlayer1_);
}


void MainWindow::CreateDialogs()
{
	dialog_save_ = new QDialog(this);
	dialog_save_->setWindowTitle(tr("Save"));
	dialog_save_->setGeometry(400, 300, 200, 200);

	QVBoxLayout *layout_save = new QVBoxLayout;
	layout_save->addWidget(groupbox_saveinfo_);
	layout_save->addWidget(groupbox_savelayer_);
	layout_save->addWidget(pushbutton_save_);
	dialog_save_->setLayout(layout_save);

	dialog_export_ = new QDialog(this);
	dialog_export_->setWindowTitle(tr("Export"));
	dialog_export_->setGeometry(300, 200, 300, 300);

	QVBoxLayout *layout_export = new QVBoxLayout;
	layout_export->addWidget(groupbox_exportvert_);
	layout_export->addWidget(groupbox_exportline_);
	layout_export->addWidget(groupbox_exportpath_);
	layout_export->addWidget(groupbox_exportlayer_);
	layout_export->addWidget(pushbutton_export_);
	dialog_export_->setLayout(layout_export);
}


void MainWindow::keyPressEvent(QKeyEvent *e)
{

}


void MainWindow::keyReleaseEvent(QKeyEvent *e)
{

}


void MainWindow::OpenFile()
{

}


void MainWindow::ChooseBaseClicked(bool down)
{
	toolbutton_choosebase_->setDown(down);
}


void MainWindow::ChooseCeilingClicked(bool down)
{
	toolbutton_chooseb_ceiling_->setDown(down);
}


void MainWindow::ChooseSubGClicked(bool down)
{
	//toolbutton_choosesubg_->setDown(down);
}


void MainWindow::GetFrameFabParas()
{
	Q_EMIT(SendFrameFabParas(
		spinbox_wp_->value(),
		spinbox_wa_->value(),
		spinbox_wi_->value(),
		action_terminal_->isChecked(),
		action_file_->isChecked()
		));
}

void MainWindow::GetFrameFabCutParas()
{
	Q_EMIT(SendFrameFabCutParas(
		spinbox_wp_->value(),
		spinbox_wa_->value(),
		spinbox_wi_->value(),
		action_terminal_->isChecked(),
		action_file_->isChecked()
		));
}

void MainWindow::GetOneLayerSearchParas()
{
	Q_EMIT(SendOneLayerSearchParas(
		spinbox_wp_->value(),
		spinbox_wa_->value(),
		spinbox_wi_->value()));
}

void MainWindow::GetDeformParas()
{
	Q_EMIT(SendDeformParas(
		spinbox_wp_->value(),
		spinbox_wa_->value(),
		spinbox_wi_->value()));
}


void MainWindow::GetProjectionParas()
{
	Q_EMIT(SendProjectionParas(spinbox_prolen_->value()));
}


void MainWindow::GetSaveParas()
{
	dialog_save_->close();
	Q_EMIT(SendSavePWFParas(
		checkbox_savevert_->isChecked(),
		checkbox_saveline_->isChecked(),
		checkbox_savepillar_->isChecked(),
		checkbox_saveceiling_->isChecked(),
		checkbox_savecut_->isChecked(),
		spinbox_minlayer1_->value(),
		spinbox_maxlayer1_->value(),
		lineedit_pwfpath_->text()));
}


void MainWindow::GetExportParas()
{
	Q_EMIT(SendExportParas(
		spinbox_minlayer2_->value(),
		spinbox_maxlayer2_->value(),
		lineedit_vertpath_->text(),
		lineedit_linepath_->text(),
		lineedit_renderpath_->text())
	);
}


void MainWindow::GetPath()
{
	if (sender() == pushbutton_exportvert_)
	{
		QString filename = QFileDialog::
			getSaveFileName(this, tr("Export Vertex"),
			"..", tr("Frame Vertex(*.txt)"));

		if (filename.isEmpty())
		{
			return;
		}

		lineedit_vertpath_->setText(filename);
	}
	else
	if (sender() == pushbutton_exportline_)
	{
		QString filename = QFileDialog::
			getSaveFileName(this, tr("Export Line"),
			"..", tr("Frame Line(*.txt)"));

		if (filename.isEmpty())
		{
			return;
		}

		lineedit_linepath_->setText(filename);
	}
	else
	{
		QString dirname = QFileDialog::
			getExistingDirectory(this,
			tr("Export Directory"),
			"/home",
			QFileDialog::ShowDirsOnly
			| QFileDialog::DontResolveSymlinks);

		if (dirname.isEmpty())
		{
			return;
		}

		lineedit_renderpath_->setText(dirname);
	}
}


void MainWindow::CheckEdgeMode()
{
	if (groupbox_edge_->isChecked())
	{
		if (sender() == radiobutton_heat_)
		{
			if (edge_render_ != HEAT)
			{
				radiobutton_heat_->setChecked(true);
				edge_render_ = HEAT;
				Q_EMIT(ChangeEdgeMode(HEAT));

				groupbox_orderdisplay_->setVisible(false);
				groupbox_edit_->setVisible(true);

				return;
			}
		}
		else
		if (sender() == radiobutton_order_)
		{
			if (edge_render_ != ORDER)
			{
				radiobutton_order_->setChecked(true);
				edge_render_ = ORDER;
				Q_EMIT(ChangeEdgeMode(ORDER));

				groupbox_edit_->setVisible(false);
				groupbox_orderdisplay_->setVisible(true);

				return;
			}
		}

		radiobutton_none_->setChecked(true);
		edge_render_ = EDGE;
		Q_EMIT(ChangeEdgeMode(EDGE));

		groupbox_orderdisplay_->setVisible(false);
		groupbox_edit_->setVisible(true);
	}
	else
	{
		radiobutton_none_->setChecked(true);
		edge_render_ = NONE;
		Q_EMIT(ChangeEdgeMode(NONE));

		groupbox_orderdisplay_->setVisible(false);
		groupbox_edit_->setVisible(true);
	}
}


void MainWindow::SwitchParaBox()
{
	//if (sender() == pushbutton_rightarrow_)
	//{
	//	groupbox_meshpara_->setVisible(false);
	//	groupbox_seqpara_->setVisible(false);
	//	groupbox_debug_->setVisible(true);
	//}
	//else
	//{
	//	groupbox_debug_->setVisible(false);
	//	groupbox_meshpara_->setVisible(true);
	//	groupbox_seqpara_->setVisible(true);
	//}
}


void MainWindow::SetOrderSlider(int value)
{
	slider_order_->setValue(value);
}


void MainWindow::SetMaxOrderSlider(int max_value)
{
	slider_order_->setMaximum(max_value);
}



void MainWindow::SetMinLayer(int min_value)
{
	spinbox_minlayer1_->setMaximum(min_value);
	spinbox_minlayer2_->setMaximum(min_value);
}


void MainWindow::SetMaxLayer(int max_value)
{
	spinbox_maxlayer1_->setMaximum(max_value);
	spinbox_maxlayer2_->setMaximum(max_value);
	spinbox_minlayer1_->setMaximum(spinbox_maxlayer1_->value());
	spinbox_minlayer2_->setMaximum(spinbox_maxlayer2_->value());
}


void MainWindow::OpenSaveDialog()
{
	QString selected_filter;
	QString filename = QFileDialog::getSaveFileName(
		this, 
		tr("Save Mesh"),
		"..", 
		tr("OBJ files(*.obj);;PWF files(*.pwf)"),
		&selected_filter
		);

	if (filename.isEmpty())
	{
		return;
	}

	if (selected_filter == "OBJ files(*.obj)")
	{
		Q_EMIT(SendSaveOBJParas(filename));
	}
	else
	{
		lineedit_pwfpath_->setText(filename);
		dialog_save_->show();
	}
}


void MainWindow::OpenExportDialog()
{
	dialog_export_->show();
}


void MainWindow::ShowMeshInfo(int npoint, int nedge)
{
	label_meshinfo_->setText(QString("MeshInfo: p: %1 e: %2").arg(npoint).arg(nedge));
}


void MainWindow::ShowCapturedVert(int id, int degree)
{
	if (id != -1)
	{
		label_capture_->setText(QString("Captured vertex: %1  Degree: %2").arg(id).arg(degree));
		label_capture_->setVisible(true);
	}
	else
	{
		label_capture_->setVisible(false);
	}
}


void MainWindow::ShowCapturedEdge(int id, double len)
{
	if (id != -1)
	{
		label_capture_->setText(QString("Captured edge: %1  Length: %2").arg(id).arg(len));
		label_capture_->setVisible(true);
	}
	else
	{
		label_capture_->setVisible(false);
	}
}


void MainWindow::ShowScale(double scale)
{
	label_operatorinfo_->setText(QString("Scale: %1").arg(scale));
}


void MainWindow::ShowLayerInfo(int layer_id, int total_id)
{
	if (layer_id <= 0)
	{
		if (total_id <= 0)
		{
			label_layer_->setVisible(false);
		}
		else
		{
			label_layer_->setText(QString("Total layer: %1").arg(total_id));
			label_layer_->setVisible(true);
		}
	}
	else
	{
		label_layer_->setText(QString("Layer: %1 / %2").arg(layer_id).arg(total_id));
		label_layer_->setVisible(true);
	}
}


void MainWindow::ShowAbout()
{
}


void MainWindow::ShowError(QString error_msg)
{
	QMessageBox::information(
		this, 
		"Error",
		error_msg,
		QMessageBox::Ok);
}


void MainWindow::Reset()
{
	//slider_layer_->setValue(0);
	slider_order_->setValue(0);

	label_operatorinfo_->setText(QString("Scale: 1.0"));

	groupbox_edge_->setChecked(false);
	radiobutton_none_->setChecked(true);
	edge_render_ = NONE;

	groupbox_orderdisplay_->setVisible(false);
	groupbox_edit_->setVisible(true);

	//groupbox_debug_->setVisible(false);
	groupbox_seqpara_->setVisible(true);
	groupbox_meshpara_->setVisible(true);
}
/*
void MainWindow::SetSlider()
{
	bool ok;
	int max_range = QInputDialog::getInt(
					this, 
					tr("Maximum Layers"), 
					tr("Please input the maximum layers"), 
					slider_layer_->maximum(), 0, 100, 1, &ok);
	if (ok)
	{
		slider_layer_->setMaximum(max_range);

	}
}
*/
