#include <framefab_gui/input_ui/renderingwidget.h>

RenderingWidget::RenderingWidget(QWidget *parent, MainWindow* mainwindow)
: QGLWidget(parent), ptr_mainwindow_(mainwindow), eye_distance_(10.0),
has_lighting_(false), is_draw_point_(true), is_draw_edge_(false), is_draw_heat_(false),
is_draw_axes_(false), op_mode_(NORMAL), scale_(1.0)
{
	ptr_arcball_ = new CArcBall(width(), height());
	ptr_frame_ = NULL;

	eye_goal_[0] = eye_goal_[1] = eye_goal_[2] = 0.0;
	eye_direction_[0] = eye_direction_[1] = 0.0;
	eye_direction_[2] = 1.0;

	last_file_dir_ = "/home";
	last_result_dir_ = "/home";

	setFocusPolicy(Qt::StrongFocus);
}


RenderingWidget::~RenderingWidget()
{
	delete (ptr_arcball_);

	if(!ptr_frame_)
	{
		delete (ptr_frame_);
	}
}


void RenderingWidget::InitDrawData()
{
	is_draw_edge_ = false;
	is_draw_heat_ = false;
	is_draw_order_ = false;
}


void RenderingWidget::InitCapturedData()
{
	op_mode_ = NORMAL;

	int N = ptr_frame_->SizeOfVertList();
	int M = ptr_frame_->SizeOfEdgeList();
	captured_verts_.clear();
	is_captured_vert_.resize(N);
	fill(is_captured_vert_.begin(), is_captured_vert_.end(), false);
	captured_edges_.clear();
	is_captured_edge_.resize(M);
	fill(is_captured_edge_.begin(), is_captured_edge_.end(), false);

	Q_EMIT(CapturedVert(-1, -1));
}


void RenderingWidget::InitFiberData()
{
	print_order_ = 0;
}


void RenderingWidget::InitInfoData(
	int vert_size, int edge_size, 
	QString oper_info,
	QString mode_info, 
	int max_slider,
	int layer_id, int total_id)
{
	Q_EMIT(ChooseBasePressed(false));
	Q_EMIT(ChooseCeilingPressed(false));
	Q_EMIT(ChooseSubGPressed(false));

	Q_EMIT(operatorInfo(oper_info));
	Q_EMIT(modeInfo(mode_info));
	Q_EMIT(meshInfo(vert_size, edge_size));

	Q_EMIT(SetMaxOrderSlider(max_slider));

	Q_EMIT(layerInfo(layer_id, total_id));
	Q_EMIT(SetMaxLayer(total_id));

	Q_EMIT(Reset());
}


void RenderingWidget::initializeGL()
{
	glClearColor(0.3, 0.3, 0.3, 0.0);
	glShadeModel(GL_SMOOTH);

	glEnable(GL_DOUBLEBUFFER);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_DEPTH_TEST);
	glClearDepth(1);

	SetLight();

}


void RenderingWidget::resizeGL(int w, int h)
{
	h = (h == 0) ? 1 : h;

	ptr_arcball_->reSetBound(w, h);

	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(45.0, GLdouble(w) / GLdouble(h), 0.001, 1000);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}


void RenderingWidget::paintGL()
{
	glShadeModel(GL_SMOOTH);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	if (has_lighting_)
	{
		SetLight();
	}
	else
	{
		glDisable(GL_LIGHTING);
		glDisable(GL_LIGHT0);
	}

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	register vec eyepos = eye_distance_*eye_direction_;
	gluLookAt(eyepos[0], eyepos[1], eyepos[2],
		eye_goal_[0], eye_goal_[1], eye_goal_[2],
		0.0, 1.0, 0.0);
	glPushMatrix();

	glMultMatrixf(ptr_arcball_->GetBallMatrix());

	Render();
	glPopMatrix();

}

void RenderingWidget::timerEvent(QTimerEvent * e)
{
	updateGL();
}


void RenderingWidget::mousePressEvent(QMouseEvent *e)
{
	switch (e->button())
	{
	case Qt::LeftButton:
		ptr_arcball_->MouseDown(e->pos());
		if (op_mode_ == CHOOSEBASE)
		{
			CaptureVertex(e->pos());
		}
		else
		if (op_mode_ == CHOOSECEILING)
		{
			CaptureEdge(e->pos());
		}
		else
		{
			if (!CaptureVertex(e->pos()))
			{
				CaptureEdge(e->pos());
			}
		}
		break;
	case Qt::MidButton:
		current_position_ = e->pos();
		break;
	case Qt::RightButton:
		if (op_mode_ == CHOOSEBASE)
		{
			if (captured_verts_.size() > 0)
			{
				WF_vert *u = captured_verts_[captured_verts_.size() - 1];
				captured_verts_.pop_back();
				is_captured_vert_[u->ID()] = false;
			}
		}
		else
		if (op_mode_ == CHOOSECEILING || op_mode_ == CHOOSESUBG)
		{
			if (captured_edges_.size() > 0)
			{
				WF_edge *e = captured_edges_[captured_edges_.size() - 1];
				captured_edges_.pop_back();
				is_captured_edge_[e->ID()] = false;
				is_captured_edge_[e->ppair_->ID()] = false;
			}
		}
		else
		{
		}
		break;
	default:
		break;
	}

	updateGL();
}


void RenderingWidget::mouseMoveEvent(QMouseEvent *e)
{
	switch (e->buttons())
	{
		setCursor(Qt::ClosedHandCursor);
	case Qt::LeftButton:
		ptr_arcball_->MouseMove(e->pos());
		break;
	case Qt::MidButton:
		eye_goal_[0] -= 4.0*GLfloat(e->x() - current_position_.x()) / GLfloat(width());
		eye_goal_[1] += 4.0*GLfloat(e->y() - current_position_.y()) / GLfloat(height());
		current_position_ = e->pos();
		break;
	default:
		break;
	}

	updateGL();
}


void RenderingWidget::mouseDoubleClickEvent(QMouseEvent *e)
{
	switch (e->button())
	{
	case Qt::LeftButton:
		break;
	default:
		break;
	}
	updateGL();
}


void RenderingWidget::mouseReleaseEvent(QMouseEvent *e)
{
	switch (e->button())
	{
	case Qt::LeftButton:
		ptr_arcball_->MouseUp(e->pos());
		setCursor(Qt::ArrowCursor);
		break;

	case Qt::RightButton:
		break;
	default:
		break;
	}
}


void RenderingWidget::wheelEvent(QWheelEvent *e)
{
	eye_distance_ += e->delta()*0.005;
	eye_distance_ = eye_distance_ < 0 ? 0 : eye_distance_;

	updateGL();
}


void RenderingWidget::keyPressEvent(QKeyEvent *e)
{
	switch (e->key())
	{
	case Qt::Key_C:
		//SwitchToChooseBound();
		break;
	case Qt::Key_Escape:
		SwitchToNormal();
		break;
	default:
		break;
	}
	updateGL();
}


void RenderingWidget::keyReleaseEvent(QKeyEvent *e)
{
	switch (e->key())
	{
	case Qt::Key_A:
		break;
	default:
		break;
	}
}


void RenderingWidget::Render()
{
	DrawAxes(is_draw_axes_);
	DrawPoints(is_draw_point_);
	DrawEdge(is_draw_edge_);
	DrawHeat(is_draw_heat_);
	DrawOrder(is_draw_order_);
}


void RenderingWidget::SetLight()
{
	static GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	static GLfloat mat_shininess[] = { 50.0 };
	static GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
	static GLfloat white_light[] = { 0.8, 0.8, 0.8, 1.0 };
	static GLfloat lmodel_ambient[] = { 0.3, 0.3, 0.3, 1.0 };

	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light);
	glLightfv(GL_LIGHT0, GL_SPECULAR, white_light);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
}


void RenderingWidget::DrawAxes(bool bV)
{
	if (!bV)
		return;
	//x axis
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0.7, 0.0, 0.0);
	glEnd();
	glPushMatrix();
	glTranslatef(0.7, 0, 0);
	glRotatef(90, 0.0, 1.0, 0.0);
	glutSolidCone(0.02, 0.06, 20, 10);
	glPopMatrix();

	//y axis
	glColor3f(0.0, 1.0, 0.0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0.0, 0.7, 0.0);
	glEnd();

	glPushMatrix();
	glTranslatef(0.0, 0.7, 0);
	glRotatef(90, -1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.06, 20, 10);
	glPopMatrix();

	//z axis
	glColor3f(0.0, 0.0, 1.0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0.0, 0.0, 0.7);
	glEnd();
	glPushMatrix();
	glTranslatef(0.0, 0, 0.7);
	glutSolidCone(0.02, 0.06, 20, 10);
	glPopMatrix();

	glColor3f(1.0, 1.0, 1.0);
}


void RenderingWidget::DrawPoints(bool bv)
{
	if (!bv || ptr_frame_ == NULL || ptr_frame_->SizeOfVertList() == 0)
	{
		return;
	}

	const std::vector<WF_vert*>& verts = *(ptr_frame_->GetVertList());
	int N = ptr_frame_->SizeOfVertList();

	glPointSize(5.0f);
	glBegin(GL_POINTS);

	for (size_t i = 0; i <N; i++)
	{
		glColor3f(1.0, 1.0, 1.0);

		if (verts[i]->isFixed())
		{
			glColor3f(0.0, 1.0, 1.0);
		}

		if (verts[i]->isBase())
		{
			glColor3f(0.0, 0.0, 1.0);
		}

		if (verts[i]->isSubgraph())
		{
			glColor3f(0.0, 1.0, 0.0);
		}

		if (is_captured_vert_[i])
		{
			glColor3f(1.0, 0.0, 0.0);
		}

		glVertex3fv(verts[i]->RenderPos().data());
	}

	glColor3f(1.0, 1.0, 1.0);
	glEnd();
}


void RenderingWidget::DrawEdge(bool bv)
{
	if (!bv || ptr_frame_ == NULL || ptr_frame_->SizeOfEdgeList() == 0)
	{
		return;
	}

	const std::vector<WF_edge*>& edges = *(ptr_frame_->GetEdgeList());
	int M = ptr_frame_->SizeOfEdgeList();

	for (size_t i = 0; i < M; i++)
	{
		WF_edge *e = edges[i];
		WF_edge *e_pair = edges[i]->ppair_;

		if (e->ID() < e_pair->ID())
		{
			glBegin(GL_LINE_LOOP);

			glColor3f(1.0, 1.0, 1.0);

			if (e->isPillar())
			{
				glColor3f(0.0, 1.0, 1.0);
			}

			if (e->isCeiling())
			{
				glColor3f(1.0, 1.0, 0.0);
			}

			if (e->isSubgraph())
			{
				glColor3f(0.0, 1.0, 0.0);
			}

			if (is_captured_edge_[i])
			{
				glColor3f(1.0, 0.0, 0.0);
			}

			glVertex3fv(e->pvert_->RenderPos().data());
			glVertex3fv(e->ppair_->pvert_->RenderPos().data());

			glEnd();
		}
	}
}


void RenderingWidget::DrawHeat(bool bv)
{
	if (!bv || ptr_frame_ == NULL)
	{
		return;
	}

	const std::vector<WF_edge*>& edges = *(ptr_frame_->GetEdgeList());
	int M = ptr_frame_->SizeOfEdgeList();

	for (int i = 0; i < M; i++)
	{
		int j = edges[i]->ppair_->ID();
		if (i < j)
		{
			WF_edge *e = edges[i];
			WF_edge *e_pair = edges[i]->ppair_;

			glBegin(GL_LINE_LOOP);

			int tag = (e->Layer() % 6);
			switch (tag)
			{
			case 0:
				glColor3f(1.0, 0.0, 0.0);
				break;
			case 1:
				glColor3f(0.0, 1.0, 0.0);
				break;
			case 2:
				glColor3f(0.0, 0.0, 1.0);
				break;
			case 3:
				glColor3f(1.0, 1.0, 0.0);
				break;
			case 4:
				glColor3f(1.0, 0.0, 1.0);
				break;
			case 5:
				glColor3f(1.0, 1.0, 1.0);
				break;

			default:
				break;
			}

			glVertex3fv(e->pvert_->RenderPos().data());
			glVertex3fv(e->ppair_->pvert_->RenderPos().data());

			glEnd();
		}
	}
}


void RenderingWidget::DrawOrder(bool bv)
{
//	if (!bv || ptr_frame_ == NULL || ptr_fiberprint_ == NULL
//		|| ptr_fiberprint_->ptr_seqanalyzer_ == NULL)
//	{
//		return;
//	}

	if (op_mode_ == NORMAL)
	{
		vector<WF_edge*> print_queue;
//		ptr_fiberprint_->OutputPrintOrder(print_queue);
		for (int i = 0; i < print_order_; i++)
		{
			WF_edge *e = print_queue[i];
			glBegin(GL_LINE_LOOP);
			glColor3f(1.0, 1.0, 1.0);
			glVertex3fv(e->pvert_->RenderPos().data());
			glVertex3fv(e->ppair_->pvert_->RenderPos().data());
			glEnd();
		}

		if (print_order_ > 0)
		{
			//ptr_fiberprint_->ptr_seqanalyzer_->GetExtru(print_order_ - 1).Render(ptr_frame_, 0.5);
		}
	}

}


void RenderingWidget::CoordinatesTransform(QPoint p, double *objx, double *objy, double *objz)
{
	double modelview[16];
	double projection[16];
	int viewport[4];

	glPushMatrix();

	glMultMatrixf(ptr_arcball_->GetBallMatrix());

	// Read the projection, modelview and viewport matrices using the glGet functions.
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetIntegerv(GL_VIEWPORT, viewport);

	glPopMatrix();

	// Read the window z value from the z-buffer 
	double winx = p.rx();
	double winy = viewport[3] - p.ry();
	float winz = 1.0;
	glReadPixels(winx, winy, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winz);

	// Use the gluUnProject to get the world co-ordinates of the point the user clicked and save in objx, objy, objz.
	gluUnProject(winx, winy, winz, modelview, projection, viewport, objx, objy, objz);
}


bool RenderingWidget::CaptureVertex(QPoint mouse)
{
	if (ptr_frame_ == NULL)
	{
		return 0;
	}

	double x = 0;
	double y = 0;
	double z = 0;
	CoordinatesTransform(mouse, &x, &y, &z);

	vector<WF_vert*> verts = *(ptr_frame_->GetVertList());
	int N = verts.size();
	for (int i = 0; i < N; i++)
	{
		double dx = verts[i]->RenderPos().x() - x;
		double dy = verts[i]->RenderPos().y() - y;
		double dz = verts[i]->RenderPos().z() - z;
		double dis = sqrt(dx*dx + dy*dy + dz*dz);
		if (dis < 0.015)
		{
			if (op_mode_ == NORMAL)
			{
				captured_verts_.clear();
				fill(is_captured_vert_.begin(), is_captured_vert_.end(), false);
				captured_edges_.clear();
				fill(is_captured_edge_.begin(), is_captured_edge_.end(), false);
			}

			if (!is_captured_vert_[i])
			{
				captured_verts_.push_back(verts[i]);
				is_captured_vert_[i] = true;
				Q_EMIT(CapturedVert(i + 1, verts[i]->Degree()));
				Q_EMIT(layerInfo(-1, -1));
			}

			return true;
		}
	}

	return false;
}


bool RenderingWidget::CaptureEdge(QPoint mouse)
{
	if (ptr_frame_ == NULL)
	{
		return 0;
	}

	double x = 0;
	double y = 0;
	double z = 0;
	CoordinatesTransform(mouse, &x, &y, &z);

	vector<WF_edge*> edges = *(ptr_frame_->GetEdgeList());
	int M = edges.size();
	for (size_t i = 0; i < M; i++)
	{
		if (edges[i]->ID() < edges[i]->ppair_->ID())
		{
			WF_vert	u = WF_vert(x, y, z);
			WF_vert *v1 = edges[i]->pvert_;
			WF_vert *v2 = edges[i]->ppair_->pvert_;

			double delta = ptr_frame_->ArcHeight(u.RenderPos(), v1->RenderPos(), v2->RenderPos());
			if (delta < 0.007)
			{
				if (op_mode_ == NORMAL)
				{
					captured_verts_.clear();
					fill(is_captured_vert_.begin(), is_captured_vert_.end(), false);
					captured_edges_.clear();
					fill(is_captured_edge_.begin(), is_captured_edge_.end(), false);
				}
	
				if (!is_captured_edge_[i])
				{
					captured_edges_.push_back(edges[i]);
					is_captured_edge_[i] = true;
				
					is_captured_edge_[edges[i]->ppair_->ID()] = true;
					Q_EMIT(CapturedEdge(i / 2, edges[i]->Length()));
					Q_EMIT(layerInfo(edges[i]->Layer() + 1, ptr_frame_->SizeOfLayer()));
				}

				return true;
			}
		}
	}

	return false;
}


void RenderingWidget::SetBackground()
{
	QColor color = QColorDialog::getColor(Qt::white, this, tr("background color"));
	GLfloat r = (color.red()) / 255.0f;
	GLfloat g = (color.green()) / 255.0f;
	GLfloat b = (color.blue()) / 255.0f;
	GLfloat alpha = color.alpha() / 255.0f;
	glClearColor(r, g, b, alpha);
	updateGL();
}


void RenderingWidget::CheckDrawPoint(bool bv)
{
	is_draw_point_ = bv;
	updateGL();
}


void RenderingWidget::CheckEdgeMode(int type)
{
	switch (type)
	{
	case NONE:
		is_draw_edge_ = false;
		is_draw_heat_ = false;
		is_draw_order_ = false;
		break;

	case EDGE:
		is_draw_edge_ = true;
		is_draw_heat_ = false;
		is_draw_order_ = false;
		break;

	case HEAT:
		is_draw_edge_ = false;
		is_draw_heat_ = true;
		is_draw_order_ = false;
		break;

	case ORDER:
		is_draw_edge_ = false;
		is_draw_heat_ = false;
		is_draw_order_ = true;
		break;

	default:
		break;
	}
	updateGL();
}


void RenderingWidget::CheckLight(bool bv)
{
	has_lighting_ = bv;
	updateGL();
}


void RenderingWidget::CheckDrawAxes(bool bV)
{
	is_draw_axes_ = bV;
	updateGL();
}


void RenderingWidget::SwitchToNormal()
{
	Q_EMIT(ChooseBasePressed(false));
	Q_EMIT(ChooseCeilingPressed(false));
	Q_EMIT(ChooseSubGPressed(false));

	if (ptr_frame_ == NULL)
	{
		Q_EMIT(modeInfo(QString("")));
	}
	else
	{
		Q_EMIT(modeInfo(QString("Choose base (B) | Choose ceiling (C)")));
	}

	if (op_mode_ == CHOOSEBASE)
	{
		ptr_frame_->MakeBase(captured_verts_);
	}
	else
		if (op_mode_ == CHOOSECEILING)
		{
			ptr_frame_->MakeCeiling(captured_edges_);
		}
		else
			if (op_mode_ == CHOOSESUBG)
			{
				ptr_frame_->MakeSubGraph(captured_edges_);
			}

	InitCapturedData();

	updateGL();
}


void RenderingWidget::SwitchToChooseBase()
{
	if (ptr_frame_ == NULL)
	{
		return;
	}

	if (op_mode_ == CHOOSEBASE)
	{
		SwitchToNormal();
	}
	else
	{
		Q_EMIT(ChooseCeilingPressed(false));
		Q_EMIT(ChooseSubGPressed(false));
		Q_EMIT(CapturedVert(-1, -1));

		Q_EMIT(ChooseBasePressed(true));
		Q_EMIT(modeInfo(QString("Choosing base...Press again or press ESC to exit.")));
		op_mode_ = CHOOSEBASE;
	}

	updateGL();
}


void RenderingWidget::SwitchToChooseCeiling()
{
	if (ptr_frame_ == NULL)
	{
		return;
	}

	if (op_mode_ == CHOOSECEILING)
	{
		SwitchToNormal();
	}
	else
	{
		Q_EMIT(ChooseBasePressed(false));
		Q_EMIT(ChooseSubGPressed(false));
		Q_EMIT(CapturedVert(-1, -1));

		Q_EMIT(ChooseCeilingPressed(true));
		Q_EMIT(modeInfo(QString("Choosing ceiling...Press again or press ESC to exit.")));
		op_mode_ = CHOOSECEILING;
	}

	updateGL();
}


void RenderingWidget::SwitchToChooseSubG()
{
	if (ptr_frame_ == NULL)
	{
		return;
	}

	if (op_mode_ == CHOOSESUBG)
	{
		SwitchToNormal();
	}
	else
	{
		Q_EMIT(ChooseBasePressed(false));
		Q_EMIT(ChooseCeilingPressed(false));
		Q_EMIT(CapturedVert(-1, -1));

		Q_EMIT(ChooseSubGPressed(true));
		Q_EMIT(modeInfo(QString("Choosing subgraph...Press again or press ESC to exit.")));
		op_mode_ = CHOOSESUBG;
	}

	updateGL();
}


void RenderingWidget::ReadFrame()
{
	QString filename = QFileDialog::getOpenFileName(
		this, 
		tr("Read Mesh"),
		last_file_dir_,
		tr("Mesh files(*.obj *.pwf)")
		);

	if (filename.isEmpty())
	{
		Q_EMIT(operatorInfo(QString("Read Mesh Failed!")));
		return;
	}

	last_file_dir_ = filename;

	// compatible with paths in chinese
	QTextCodec *code = QTextCodec::codecForName("gd18030");
	QTextCodec::setCodecForLocale(code);
	QByteArray byfilename = filename.toLocal8Bit();

	delete ptr_frame_; 
	ptr_frame_ = new WireFrame();

	if (filename.contains(".obj") || filename.contains(".OBJ"))
	{
		ptr_frame_->LoadFromOBJ(byfilename.data());
	}
	else
	{
		ptr_frame_->LoadFromPWF(byfilename.data());
	}


	InitDrawData();
	InitCapturedData();
	InitFiberData();
	InitInfoData(
		ptr_frame_->SizeOfVertList(), ptr_frame_->SizeOfEdgeList() / 2,
		QString(""),
		QString("Choose base (B) | Choose ceiling (C)"),
		ptr_frame_->SizeOfEdgeList() / 2,
		-1, ptr_frame_->SizeOfLayer()
	);

	
	updateGL();
}


void RenderingWidget::WriteFrame(QString filename)
{
	if (ptr_frame_ == NULL || ptr_frame_->SizeOfVertList() == 0)
	{
		Q_EMIT(QString("The mesh is Empty !"));
		return;
	}

	// compatible with paths in chinese
	QTextCodec *code = QTextCodec::codecForName("gd18030");
	QTextCodec::setCodecForLocale(code);
	QByteArray byfilename = filename.toLocal8Bit();

	ptr_frame_->WriteToOBJ(filename.toLatin1().data());

	Q_EMIT(operatorInfo(QString("Write mesh to ") + filename + QString(" Done")));
}


void RenderingWidget::WriteFrame(
	bool bVert, bool bLine, 
	bool bPillar, bool bCeiling, 
	bool bCut, int min_layer, int max_layer, 
	QString filename)
{
	if (ptr_frame_ == NULL || ptr_frame_->SizeOfVertList() == 0)
	{
		Q_EMIT(QString("The mesh is Empty !"));
		return;
	}

	// compatible with paths in chinese
	QTextCodec *code = QTextCodec::codecForName("gd18030");
	QTextCodec::setCodecForLocale(code);
	QByteArray byfilename = filename.toLocal8Bit();

	ptr_frame_->WriteToPWF(
		bVert, bLine, 
		bPillar, bCeiling,
		bCut, min_layer, max_layer, 
		filename.toLatin1().data()
	);

	Q_EMIT(operatorInfo(QString("Write mesh to ") + filename + QString(" Done")));
}


void RenderingWidget::Import()
{
	QString selected_filter;
	QString filename = QFileDialog::getOpenFileName(
		this, 
		tr("Import"),
		last_file_dir_,
		tr("3DD files(*.3dd);;Sequence files(*.txt)"),
		&selected_filter
		);

	if (filename.isEmpty())
	{
		Q_EMIT(operatorInfo(QString("Import Failed!")));
		return;
	}

	last_file_dir_ = filename;

	// compatible with paths in chinese
	QTextCodec *code = QTextCodec::codecForName("gd18030");
	QTextCodec::setCodecForLocale(code);
	QByteArray byfilename = filename.toLocal8Bit();

	int M = 0;
	if (selected_filter == "3DD files(*.3dd)")
	{
		delete ptr_frame_;
		ptr_frame_ = new WireFrame();

		ptr_frame_->ImportFrom3DD(byfilename.data());
	}
	else
	{
		if (ptr_frame_ == NULL || ptr_frame_->SizeOfVertList() == 0)
		{
			Q_EMIT(QString("The mesh is Empty !"));
			return;
		}

//		if (ptr_fiberprint_ == NULL)
//		{
//			FiberPrintPARM *ptr_parm = new FiberPrintPARM();
//			ptr_fiberprint_ = new FiberPrintPlugIn(ptr_frame_, ptr_parm, byfilename.data());
//		}
//
//		if (!ptr_fiberprint_->ImportPrintOrder(byfilename.data()))
//		{
//			Q_EMIT(QString("Import failed!"));
//		}
//		else
//		{
//			M = ptr_frame_->SizeOfEdgeList() / 2;
//		}
	}


	InitDrawData();
	InitCapturedData();
	InitFiberData();
	InitInfoData(
		ptr_frame_->SizeOfVertList(), ptr_frame_->SizeOfEdgeList() / 2,
		QString(""),
		QString("Choose base (B) | Choose ceiling (C)"),
		M,
		-1, ptr_frame_->SizeOfLayer()
	);

	updateGL();
}


void RenderingWidget::Export()
{
	if (ptr_frame_ == NULL || ptr_frame_->SizeOfVertList() == 0)
	{
		Q_EMIT(QString("The mesh is Empty !"));
		return;
	}

	QString selected_filter;
	QString filename = QFileDialog::getSaveFileName(
		this,
		tr("Export"),
		"..",
		tr("Sequence(*.txt);;Subgraph(*.obj)"),
		&selected_filter
		); 

	if (filename.isEmpty())
	{
		Q_EMIT(operatorInfo(QString("Export Failed!")));
		return;
	}

	// compatible with paths in chinese
	QTextCodec *code = QTextCodec::codecForName("gd18030");
	QTextCodec::setCodecForLocale(code);
	QByteArray byfilename = filename.toLocal8Bit();

	if (selected_filter == "Sequence(*.txt)")
	{
//		if (ptr_fiberprint_ == NULL)
//		{
//			FiberPrintPARM *ptr_parm = new FiberPrintPARM();
//			ptr_fiberprint_ = new FiberPrintPlugIn(ptr_frame_, ptr_parm, NULL);
//		}
//		ptr_fiberprint_->ExportPrintOrder(byfilename.data());
//		Q_EMIT(operatorInfo(QString("Export sequence to ") + filename + QString(" Done")));
	}
	else
	if (selected_filter == "Subgraph(*.obj)")
	{
		ptr_frame_->ExportSubgraph(byfilename.data());
		Q_EMIT(operatorInfo(QString("Export subgraph to ") + filename + QString(" Done")));
	}
}


void RenderingWidget::Export(
	int min_layer, int max_layer,
	QString vert_path, QString line_path, QString render_path
	)
{	
	if (ptr_frame_ == NULL || ptr_frame_->SizeOfVertList() == 0)
	{
		Q_EMIT(QString("The mesh is Empty !"));
		return;
	}

	if (min_layer == 0 || max_layer == 0 || min_layer > max_layer)
	{
		Q_EMIT(Error(QString("Invalid layer information")));
		return;
	}

	// compatible with paths in chinese
	QTextCodec *code = QTextCodec::codecForName("gd18030");
	QTextCodec::setCodecForLocale(code);

	if (!vert_path.isEmpty())
	{
		ptr_frame_->ExportPoints(min_layer, max_layer,
			vert_path.toLocal8Bit().data());
	}

	if (!line_path.isEmpty())
	{
		ptr_frame_->ExportLines(min_layer, max_layer,
			line_path.toLocal8Bit().data());
	}
//
//	if (!render_path.isEmpty())
//	{
//		if (ptr_fiberprint_ == NULL)
//		{
//			Q_EMIT(operatorInfo(QString("Export render path failed.")));
//			return;
//		}
//		ptr_fiberprint_->ExportRenderPath(
//			min_layer, max_layer,
//			render_path.toLocal8Bit().data()
//		);
//	}

	Q_EMIT(operatorInfo(QString("Export mesh done.")));	
}


void RenderingWidget::ScaleFrame(double scale)
{
	if (ptr_frame_ == NULL)
	{
		return;
	}

	vector<WF_vert*>& verts = *(ptr_frame_->GetVertList());
	int N = ptr_frame_->SizeOfVertList();
	float size = scale / scale_;
	for (int i = 0; i < N; i++)
	{
		Vec3f p = verts[i]->Position();
		verts[i]->SetPosition(p * size);
	}
	scale_ = scale;
	ptr_frame_->Unify();

	int cape_size = captured_edges_.size();
	if (cape_size != 0)
	{
		Q_EMIT(CapturedEdge(captured_edges_[cape_size - 1]->ID() + 1,
			captured_edges_[cape_size - 1]->Length()));
	}

	updateGL();
}


void RenderingWidget::FrameFabAnalysis(
	double Wl, double Wp, double Wa, 
	bool terminal_output, bool file_output
	)
{
//	FiberPrintPARM *ptr_parm = new FiberPrintPARM(Wl, Wp, Wa);

	if (file_output)
	{
		QString dirname = QFileDialog::getExistingDirectory(
			this,
			tr("Result Directory"),
			last_result_dir_,
			QFileDialog::ShowDirsOnly
			| QFileDialog::DontResolveSymlinks
			);

		if (dirname.isEmpty())
		{
			Q_EMIT(operatorInfo(QString("Read Directory Failed!")));
			return;
		}

		last_result_dir_ = dirname;

		// compatible with paths in chinese
		QTextCodec *code = QTextCodec::codecForName("gd18030");
		QTextCodec::setCodecForLocale(code);
		QByteArray bydirname = dirname.toLocal8Bit();

//		delete ptr_fiberprint_;
//		ptr_fiberprint_ = new FiberPrintPlugIn(
//			ptr_frame_, ptr_parm, bydirname.data(),
//			terminal_output, file_output
//			);
	}
	else
	{
//		delete ptr_fiberprint_;
//		ptr_fiberprint_ = new FiberPrintPlugIn(
//			ptr_frame_, ptr_parm, NULL,
//			terminal_output, file_output
//			);
	}

//	ptr_fiberprint_->FrameFabPrint();

	InitInfoData(
		ptr_frame_->SizeOfVertList(), ptr_frame_->SizeOfEdgeList() / 2,
		QString(""),
		QString(""),
		ptr_frame_->SizeOfEdgeList() / 2,
		-1, ptr_frame_->SizeOfLayer()
	);
}

void RenderingWidget::CutAnalysis(
	double Wl, double Wp, double Wa,
	bool terminal_output, bool file_output
	)
{
	QByteArray bydirname = NULL;
//	FiberPrintPARM *ptr_parm = new FiberPrintPARM(Wl, Wp, Wa);

	if (file_output)
	{
		QString dirname = QFileDialog::getExistingDirectory(
			this,
			tr("Result Directory"),
			last_result_dir_,
			QFileDialog::ShowDirsOnly
			| QFileDialog::DontResolveSymlinks
			);

		if (dirname.isEmpty())
		{
			Q_EMIT(operatorInfo(QString("Read Directory Failed!")));
			return;
		}

		last_result_dir_ = dirname;

		// compatible with paths in chinese
		QTextCodec *code = QTextCodec::codecForName("gd18030");
		QTextCodec::setCodecForLocale(code);
		bydirname = dirname.toLocal8Bit();

//		delete ptr_fiberprint_;
//		ptr_fiberprint_ = new FiberPrintPlugIn(
//			ptr_frame_, ptr_parm, bydirname.data(),
//			terminal_output, file_output
//			);
	}
	else
	{
//		delete ptr_fiberprint_;
//		ptr_fiberprint_ = new FiberPrintPlugIn(
//			ptr_frame_, ptr_parm, NULL,
//			terminal_output, file_output
//			);
	}

//	ptr_fiberprint_->GetFrameFabCut();

	InitInfoData(
		ptr_frame_->SizeOfVertList(), ptr_frame_->SizeOfEdgeList() / 2,
		QString(""),
		QString(""),
		ptr_frame_->SizeOfEdgeList() / 2,
		-1, ptr_frame_->SizeOfLayer()
		);
}

void RenderingWidget::OneLayerAnalysis(double Wl, double Wp, double Wa)
{
	QString dirname = QFileDialog::getExistingDirectory(
		this,
		tr("Result Directory"),
		last_result_dir_,
		QFileDialog::ShowDirsOnly
		| QFileDialog::DontResolveSymlinks
		);

	if (dirname.isEmpty())
	{
		Q_EMIT(operatorInfo(QString("Read Directory Failed!")));
		return;
	}

	last_result_dir_ = dirname;

	// compatible with paths in chinese
	QTextCodec *code = QTextCodec::codecForName("gd18030");
	QTextCodec::setCodecForLocale(code);
	QByteArray bydirname = dirname.toLocal8Bit();


//	FiberPrintPARM *ptr_parm = new FiberPrintPARM(Wl, Wp, Wa);

//	delete ptr_fiberprint_;
//	ptr_fiberprint_ = new FiberPrintPlugIn(ptr_frame_, ptr_parm, bydirname.data());
//
//	ptr_fiberprint_->OneLayerPrint();

	InitInfoData(
		ptr_frame_->SizeOfVertList(), ptr_frame_->SizeOfEdgeList() / 2,
		QString(""),
		QString(""),
		ptr_frame_->SizeOfEdgeList() / 2,
		-1, ptr_frame_->SizeOfLayer()
		);
}

void RenderingWidget::DeformationAnalysis(double Wl, double Wp, double Wa)
{
	QString dirname = QFileDialog::
		getExistingDirectory(this,
		tr("Result Directory"),
		"/home",
		QFileDialog::ShowDirsOnly
		| QFileDialog::DontResolveSymlinks);

	if (dirname.isEmpty())
	{
		Q_EMIT(operatorInfo(QString("Read Directory Failed!")));
		return;
	}

	// compatible with paths in chinese
	QTextCodec *code = QTextCodec::codecForName("gd18030");
	QTextCodec::setCodecForLocale(code);
	QByteArray bydirname = dirname.toLocal8Bit();


	FiberPrintPARM *ptr_parm = new FiberPrintPARM(Wl, Wp, Wa);

//	delete ptr_fiberprint_;
//	ptr_fiberprint_ = new FiberPrintPlugIn(ptr_frame_, ptr_parm, bydirname.data(), false, true);
//
//	ptr_fiberprint_->GetDeformation();

	Q_EMIT(SetOrderSlider(0));
	Q_EMIT(SetMaxOrderSlider(0));
}


void RenderingWidget::ProjectBound(double len)
{
	ptr_frame_->ProjectBound(len);

	Q_EMIT(modeInfo(QString("Choose base (B) | Choose ceiling (C)")));
	Q_EMIT(operatorInfo(QString("")));
	Q_EMIT(meshInfo(ptr_frame_->SizeOfVertList(), ptr_frame_->SizeOfEdgeList()));

	InitCapturedData();
	updateGL();
}


void RenderingWidget::ModifyProjection(double len)
{
	ptr_frame_->ModifyProjection(len);
	updateGL();
}


void RenderingWidget::RotateXY()
{
	vector<WF_vert*>& verts = *(ptr_frame_->GetVertList());
	int N = ptr_frame_->SizeOfVertList();

	for (int i = 0; i < N; i++)
	{
		verts[i]->SetPosition(verts[i]->Position().y(), verts[i]->Position().x(),
			verts[i]->Position().z());
		verts[i]->SetRenderPos(verts[i]->RenderPos().y(), verts[i]->RenderPos().x(),
			verts[i]->RenderPos().z());
	}

	ptr_frame_->Unify();
	updateGL();
}


void RenderingWidget::RotateXZ()
{
	vector<WF_vert*>& verts = *(ptr_frame_->GetVertList());
	int N = ptr_frame_->SizeOfVertList();

	for (int i = 0; i < N; i++)
	{
		verts[i]->SetPosition(verts[i]->Position().z(), verts[i]->Position().y(),
			verts[i]->Position().x());
		verts[i]->SetRenderPos(verts[i]->RenderPos().z(), verts[i]->RenderPos().y(),
			verts[i]->RenderPos().x());
	}

	ptr_frame_->Unify();
	updateGL();
}


void RenderingWidget::RotateYZ()
{
	vector<WF_vert*>& verts = *(ptr_frame_->GetVertList());
	int N = ptr_frame_->SizeOfVertList();

	for (int i = 0; i < N; i++)
	{
		verts[i]->SetPosition(verts[i]->Position().x(), verts[i]->Position().z(),
			verts[i]->Position().y());
		verts[i]->SetRenderPos(verts[i]->RenderPos().x(), verts[i]->RenderPos().z(),
			verts[i]->RenderPos().y());
	}

	ptr_frame_->Unify();
	updateGL();
}


void RenderingWidget::PrintOrder(int order)
{
	print_order_ = order;
	updateGL();
}


void RenderingWidget::PrintLastStep()
{
	if (print_order_ > 0)
	{
		print_order_--;
	}

	updateGL();

	Q_EMIT(SetOrderSlider(print_order_));
}


void RenderingWidget::PrintNextStep()
{
	int M = ptr_frame_->SizeOfEdgeList() / 2;
	if (print_order_ < M)
	{
		print_order_++;
	}

	updateGL();

	Q_EMIT(SetOrderSlider(print_order_));
}


void RenderingWidget::PrintLastLayer()
{
	vector<WF_edge*> print_queue;
//	ptr_fiberprint_->OutputPrintOrder(print_queue);

	int M = ptr_frame_->SizeOfEdgeList() / 2;
	if (print_order_ == M)
	{
		print_order_--;
	}
	int last_layer = max(0, print_queue[print_order_]->Layer() - 1);

	while (1)
	{
		if (print_order_ == 0)
		{
			break;
		}

		if (print_queue[print_order_]->Layer() == last_layer)
		{
			break;
		}
		print_order_--;
	}

	updateGL();

	Q_EMIT(SetOrderSlider(print_order_));
}


void RenderingWidget::PrintNextLayer()
{
	vector<WF_edge*> print_queue;
//	ptr_fiberprint_->OutputPrintOrder(print_queue);

	int M = ptr_frame_->SizeOfEdgeList() / 2;
	if (print_order_ == M)
	{
		print_order_--;
	}
	int next_layer = print_queue[print_order_]->Layer() + 1;

	while (1)
	{
		if (print_order_ == M)
		{
			break;
		}

		if (print_queue[print_order_]->Layer() == next_layer)
		{
			break;
		}
		print_order_++;
	}

	updateGL();

	Q_EMIT(SetOrderSlider(print_order_));
}