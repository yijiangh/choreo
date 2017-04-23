#include "NormalCut.h"


NormalCut::NormalCut()
{
}


NormalCut::NormalCut(WireFrame *ptr_frame, char *ptr_path)
{
	ptr_frame_ = ptr_frame;
	ptr_path_ = ptr_path;
}


NormalCut::~NormalCut()
{
}


void NormalCut::MakeLayers()
{
	int M = ptr_frame_->SizeOfEdgeList();

	for (int i = 0; i < M; i++)
	{
		WF_edge *e = ptr_frame_->GetEdge(i);
		if (e->ID() < e->ppair_->ID())
		{
			double lower_z = min(e->pvert_->Position().z(), 
				e->ppair_->pvert_->Position().z());
			sweep_queue_.insert(make_pair(lower_z, e));
		}
	}

	int layer_gap = 35;
	int l = 0;
	multimap<double, WF_edge*>::iterator it;
	for (it = sweep_queue_.begin(); it != sweep_queue_.end(); l++)
	{
		for (int j = 0; j < layer_gap && it != sweep_queue_.end(); j++, it++)
		{
			it->second->SetLayer(l);
			it->second->ppair_->SetLayer(l);
		}
	}

	ptr_frame_->Unify();
}