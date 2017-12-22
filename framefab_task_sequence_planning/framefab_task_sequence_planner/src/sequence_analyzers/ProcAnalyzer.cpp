#include <sequence_analyzers/ProcAnalyzer.h>

ProcAnalyzer::ProcAnalyzer()
{


}


ProcAnalyzer::~ProcAnalyzer()
{
}


ProcAnalyzer::ProcAnalyzer(SeqAnalyzer *seqanalyzer, char *path)
{
	ptr_seqanalyzer_ = seqanalyzer;
	path_ = path;
	debug_ = false;

	MaxEdgeAngle_ = F_PI / 18*6;
}


void ProcAnalyzer::ProcPrint()
{

	WireFrame *ptr_frame = ptr_seqanalyzer_->ptr_frame_;
	DualGraph *ptr_dualgraph = ptr_seqanalyzer_->ptr_dualgraph_;

	QuadricCollision *ptr_collision = new QuadricCollision(ptr_frame);

	vector<WF_edge*> print_queue;
	
	if (debug_)
	{
		ptr_dualgraph->Dualization();
		ReadLayerQueue();
	}
	else
	{
		ptr_seqanalyzer_->OutputPrintOrder(print_queue);
	}

	for (int i = 0; i < print_queue.size(); i++)
	{
		layer_queue_.push_back(print_queue[i]->ID());
	}


	exist_point_.clear();
	process_list_.clear();
	support_ = 0;

	//angle
	for (int i = 0; i < layer_queue_.size(); i++)
	{
		Process temp;
		int orig_e = layer_queue_[i];
		WF_edge *e = ptr_frame->GetEdge(orig_e);
		if (e->isPillar())
		{
			temp.normal_.push_back(GeoV3(0, 0, 1));
			support_++;
		}
		else
		{
			ptr_collision->DetectCollision(e, exist_edge_, temp.normal_);
		}
		exist_edge_.push_back(e);
		process_list_.push_back(temp);
	}

	//point
	for (int i = 0; i < layer_queue_.size(); i++)
	{
		Process temp = process_list_[i];
		int orig_e = layer_queue_[i];
		WF_edge *e = ptr_frame->GetEdge(orig_e);

		point up, down;
		if ((e->pvert_->Position()).z()>(e->ppair_->pvert_->Position()).z())
		{
			up = e->pvert_->Position();
			down = e->ppair_->pvert_->Position();
		}
		else
		{
			up = e->ppair_->pvert_->Position();
			down = e->pvert_->Position();
		}

		if (e->isPillar())
		{
			temp.start_ = down;
			temp.end_ = up;
			temp.fan_state_ = true;
			if (!IfPointInVector(down))
				exist_point_.push_back(down);
			if (!IfPointInVector(up))
				exist_point_.push_back(up);
		}
		else
		{
			if (IfPointInVector(down) && IfPointInVector(up))
			{
				temp.fan_state_ = false;
				temp.start_ = down;
				temp.end_ = up;
			}
			else if (IfPointInVector(down))
			{
				temp.fan_state_ = true;
				temp.start_ = down;
				temp.end_ = up;
				exist_point_.push_back(up);
			}
			else
			{
				temp.fan_state_ = true;
				temp.start_ = up;
				temp.end_ = down;
				exist_point_.push_back(down);
			}
		} 
		process_list_[i] = temp;
	}


	for (int i = 0; i < process_list_.size(); i++)
	{
		if (process_list_[i].fan_state_)
		{
			Fitler(process_list_[i]);
		}
		else
			CheckProcess(process_list_[i]);
	}


	Write();
	delete ptr_collision;
	ptr_collision = NULL;
}

void ProcAnalyzer::ReadLayerQueue()
{
	string path = path_;
	string queue_path = path + "/Queue.txt";

	FILE *fp = fopen(queue_path.c_str(), "r");

	layer_queue_.clear();
	int Nd = ptr_seqanalyzer_->ptr_dualgraph_->SizeOfEdgeList();
	for (int i = 0; i < Nd; i++)
	{
		int e;
		fscanf(fp, "%d", &e);
		layer_queue_.push_back(e);
	}
	std::fclose(fp);
}

bool  ProcAnalyzer::IfPointInVector(point p)
{
	for (int i = 0; i <exist_point_.size(); i++)
	{
		if ((exist_point_[i] - p).length() < GEO_EPS)
			return true;
	}
	return false;
}

void ProcAnalyzer::Write()
{
	string path = path_;
	string fan_path = path + "/FanState.txt";
	string istart_path = path + "/IStart.txt";
	string iend_path = path + "/IEnd.txt";
	string isupport_path = path + "/ISupport.txt";

	FILE *fans = fopen(fan_path.c_str(), "w+");
	FILE *start = fopen(istart_path.c_str(), "w+");
	FILE *end = fopen(iend_path.c_str(), "w+");
	FILE *support = fopen(isupport_path.c_str(), "w+");
	std::fprintf(support, "%d", support_);
	std::fclose(support);


	for (int i = 0; i < process_list_.size(); i++)
	{
		Process temp = process_list_[i];
		point p = temp.start_;
		std::fprintf(start, "%lf ,%lf ,%lf", p.x(), p.y(), p.z());
		std::fprintf(start, "\n");

		p = temp.end_;
		std::fprintf(end, "%lf ,%lf ,%lf", p.x(), p.y(), p.z());
		std::fprintf(end, "\n");

		std::fprintf(fans, "%d", temp.fan_state_);
		std::fprintf(fans, "\n");

		stringstream ss;
		string str;
		ss << i;
		ss >> str;

		string ivector_path = path + "/INormal_" + str + ".txt";
		FILE *vector = fopen(ivector_path.c_str(), "w+");
		if (temp.normal_.size() == 0)
		{
			cout << "error:normal vector empty" << endl;
			getchar();
		}
		for (int j = 0; j < temp.normal_.size(); j++)
		{
			if (temp.normal_[j].getZ() < 0)
			{
				continue;
			}
			std::fprintf(vector, "%lf ,%lf ,%lf", temp.normal_[j].getX(), temp.normal_[j].getY(), temp.normal_[j].getZ());
			std::fprintf(vector, "\n");
		}
		std::fclose(vector);
	}
	std::fclose(start);
	std::fclose(end);
	std::fclose(fans);

}


bool  ProcAnalyzer::IfCoOrientation(GeoV3 a, vector<GeoV3> &b)
{
	for (int i = 0; i < b.size(); i++)
	{
		if (angle(a, b[i]) < (F_PI / 2))
			return true;
	}
	return false;
}

void ProcAnalyzer::CheckProcess(Process &a)
{
	GeoV3 t = a.end_ - a.start_;

	t.normalize();
	vector<GeoV3> temp_normal;
	if (!IfCoOrientation(t, a.normal_))
	{
		point temp = a.end_;
		a.end_ = a.start_; 
		a.start_ = temp;
		for (int i = 0; i < a.normal_.size(); i++)
		{
			if (angle(t, a.normal_[i]) <(F_PI / 2))
				continue;
		  else
				temp_normal.push_back(a.normal_[i]);
		}		
	}
	else
	{
		for (int i = 0; i < a.normal_.size(); i++)
		{
			if (angle(t, a.normal_[i]) <(F_PI / 2))				
				temp_normal.push_back(a.normal_[i]);
		}
	}
	a.normal_ = temp_normal;

}


void ProcAnalyzer::Fitler(Process &a)
{
	GeoV3 t = a.end_ - a.start_;
	t.normalize();
	vector<GeoV3> temp;
	if (IfCoOrientation(t, a.normal_))
	{
		for (int i = 0; i < a.normal_.size(); i++)
		{
			if (angle(t, a.normal_[i]) < MaxEdgeAngle_)
			{
				temp.push_back(a.normal_[i]);
			}
		}
	}
	if (temp.size())
		a.normal_ = temp;
}


void ProcAnalyzer::CollisionColorMap()
{
	WireFrame *ptr_frame = ptr_seqanalyzer_->ptr_frame_;
	DualGraph *ptr_dualgraph = ptr_seqanalyzer_->ptr_dualgraph_;
	QuadricCollision *ptr_collision = new QuadricCollision(ptr_frame);
	int Nd = ptr_dualgraph->SizeOfVertList();

	vector<vector<double>> writer;
	double cost_max = 0;
	double cost_min = 1;

	if (debug_)
	{
		ptr_dualgraph->Dualization();
		ReadLayerQueue();
	}
	else
	{
		//ptr_seqanalyzer_->OutputPrintOrder(layer_queue_);
	}

	for (int i = 0; i < layer_queue_.size(); i++)
	{
		int orig_e = layer_queue_[i];
		WF_edge *target = ptr_frame->GetEdge(orig_e);
		if (target->isPillar())
			continue;
		char id[10];
		sprintf(id, "%d", i);
		string path = path_;
		string file = path + "/CollisionRender_" + id + ".txt";
		FILE *fp = fopen(file.c_str(), "w+");

		vector<double> cost_list;
		double cost_max = 0;
		double cost_min = 1;

		for (int j = 0; j <Nd; j++)
		{
			WF_edge *e = ptr_frame->GetEdge(  layer_queue_[j]);

			double cost;
			if (i == j)
			{
				cost = 0;
			}
			else
			{
				vector<lld> colli_map;
				ptr_collision->DetectCollision(target, e, colli_map);
				cost = 1 - (double)ptr_collision->ColFreeAngle(colli_map) / (double)ptr_collision->Divide();
				if (cost > cost_max)
					cost_max = cost;
				if (cost < cost_min)
					cost_min = cost;
			}
			cost_list.push_back(cost);
		}



		for (int j = 0; j < Nd; j++)
		{
			WF_edge *e = ptr_frame->GetEdge(layer_queue_[j]);
	
			double r;
			double g;
			double b;

			if (i == j)
			{
				r = 1;
				g = 1;
				b = 1;
			}
			else
			{

				double cost = cost_list[j];
				cost = (cost - cost_min) / (cost_max - cost_min);
				if (cost < 0.25)
				{
					r = 0.0;
					g = cost * 4.0;
					b = 1.0;
				}
				else
				if (cost < 0.5)
				{
					r = 0.0;
					g = 1.0;
					b = (0.5 - cost) * 4.0;
				}
				else
				if (cost < 0.75)
				{
					r = (cost - 0.5) * 4.0;
					g = 1.0;
					b = 0.0;
				}
				else
				{
					r = 1.0;
					g = (1.0 - cost) * 4.0;
					b = 0.0;
				}
			}

			point u = e->pvert_->RenderPos();
			point v = e->ppair_->pvert_->RenderPos();
			std::fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
				u.x(), u.y(), u.z(), v.x(), v.y(), v.z(), r, g, b);
		}
		fclose(fp);
	}
	
}


void ProcAnalyzer::CollisionColorMap(int x)
{
	WireFrame *ptr_frame = ptr_seqanalyzer_->ptr_frame_;
	DualGraph *ptr_dualgraph = ptr_seqanalyzer_->ptr_dualgraph_;
	QuadricCollision *ptr_collision = new QuadricCollision(ptr_frame);
	int Nd = (ptr_frame->GetEdgeList())->size();

	if (debug_)
	{
		ptr_dualgraph->Dualization();
		ReadLayerQueue();
	}
	else
	{
		//ptr_seqanalyzer_->OutputPrintOrder(layer_queue_);
	}
	int cout_txt = 0;
	for (int i = 0; i < Nd; i++)
	{
		int orig_e = i;
		WF_edge *target = ptr_frame->GetEdge(orig_e);

		if (target->ID() < target->ppair_->ID())
			continue;

		if (target->isPillar())
			continue;

		double cost_max = 0;
		double cost_min = 1;

		vector<vector<double>>writer;
	
		for (int j = 0; j < Nd; j++)
		{
			WF_edge *e = ptr_frame->GetEdge(j);
			if (e->ID() < e->ppair_->ID())
				continue;

			double r;
			double g;
			double b;
			double		cost;
			if (i == j)
			{
				r = 0;
				g = 0;
				b = 0;
				cost = 0;
			}
			else
			{
				vector<lld> colli_map;
				ptr_collision->DetectCollision(target, e, colli_map);
				cost = 1 - (double)ptr_collision->ColFreeAngle(colli_map) / (double)ptr_collision->Divide();	
				if (cost > cost_max)
					cost_max = cost;
				if (cost < cost_min)
					cost_min = cost;
			}
			point u = e->pvert_->RenderPos();
			point v = e->ppair_->pvert_->RenderPos();

			vector<double> temp_9;
			temp_9.push_back(u.x());
			temp_9.push_back(u.y());
			temp_9.push_back(u.z());
			temp_9.push_back(v.x());
			temp_9.push_back(v.y());
			temp_9.push_back(v.z());
			temp_9.push_back(cost);

			writer.push_back(temp_9);
		}
		char id[10];
		std::sprintf(id, "%d", cout_txt);
		cout_txt++;
		string path = path_;
		string file = path + "/CollisionRender_" + id + ".txt";
		FILE *fp = fopen(file.c_str(), "w+");
		for (int s = 0; s < writer.size(); s++)
		{
			double r, g, b;
			ColorMap((writer[s][6] - cost_min) / (cost_max - cost_min), r, g, b);
			std::fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
				writer[s][0], writer[s][1], writer[s][2], writer[s][3], writer[s][4], writer[s][5], r, g, b);
		}

		fclose(fp);
	}

}

void ProcAnalyzer::ColorMap(double cost, double &r, double &g, double &b)
{

	if (cost < 0.25)
	{
		r = 0.0;
		g = cost * 4.0;
		b = 1.0;
	}
	else
	if (cost < 0.5)
	{
		r = 0.0;
		g = 1.0;
		b = (0.5 - cost) * 4.0;
	}
	else
	if (cost < 0.75)
	{
		r = (cost - 0.5) * 4.0;
		g = 1.0;
		b = 0.0;
	}
	else
	{
		r = 1.0;
		g = (1.0 - cost) * 4.0;
		b = 0.0;
	}
}