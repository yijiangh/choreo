#include "BFAnalyzer.h"


BFAnalyzer::BFAnalyzer()
{
}


BFAnalyzer::~BFAnalyzer()
{
}


bool BFAnalyzer::SeqPrint()
{
	BF_analyzer_.Start();

	Init();

	/* set pillars as starting edges */
	PrintPillars();
	bool ret = GenerateSeq(print_queue_.size(), Nd_);
	
	BF_analyzer_.Stop();

	return ret;
}


bool BFAnalyzer::GenerateSeq(int h, int t)
{
	if (h == t)
	{
		return true;
	}

	if (terminal_output_)
	{
		fprintf(stderr, "---searching at edge %d, head %d, (tail %d).\n", 
			print_queue_[h - 1]->ID() / 2, h, t);
	}

	WF_edge *ei = print_queue_[h - 1];
	for (int dual_j = 0; dual_j < Nd_; dual_j++)
	{
		int orig_j = ptr_wholegraph_->e_orig_id(dual_j);
		WF_edge *ej = ptr_frame_->GetEdge(orig_j);
		if (!ptr_dualgraph_->isExistingEdge(ej))
		{
			if (terminal_output_)
			{
				fprintf(stderr, "###Attempting edge %d.\n", dual_j);
			}
			if (ptr_dualgraph_->isExistingVert(ej->pvert_->ID())
				|| ptr_dualgraph_->isExistingVert(ej->ppair_->pvert_->ID()))
			//if (ei->pvert_ == ej->pvert_ || ei->ppair_->pvert_ == ej->pvert_
			//	|| ei->pvert_ == ej->ppair_->pvert_ || ei->ppair_->pvert_ == ej->ppair_->pvert_)
			{
				int free_angle = ptr_collision_->ColFreeAngle(angle_state_[dual_j]);
				if (free_angle == 0)
				{
					printf("...edge %d: collision examination failed.\n", orig_j / 2);
					return false;
				}

				if (!TestifyStiffness(ej))
				{
					printf("...edge %d: Stiffness examination failed.\n", ej->ID() / 2);
					return false;
				}

				print_queue_.push_back(ej);
				UpdateStructure(ej);
				vector<vector<lld>> tmp_angle(3);
				UpdateStateMap(ej, tmp_angle);

				if (GenerateSeq(h + 1, t))
				{
					return true;
				}

				RecoverStateMap(ej, tmp_angle);
				RecoverStructure(ej);
				print_queue_.pop_back();
			}
		}
	}

	printf("No next choice.\n");
	return false;
}


void BFAnalyzer::PrintOutQueue(int N)
{	
	string path = ptr_path_;
	string queue_path = path + "/BruteForceQueue.txt";

	FILE *fp = fopen(queue_path.c_str(), "w");

	for (int i = 0; i < N; i++)
	{
		fprintf(fp, "%d\n", print_queue_[i]->ID() / 2);
	}

	fclose(fp);
}


void BFAnalyzer::PrintOutTimer()
{
	printf("***BFAnalyzer timer result:\n");
	BF_analyzer_.Print("BFAnalyzer:");
	upd_struct_.Print("UpdateStructure:");
	rec_struct_.Print("RecoverStructure:");
	upd_map_.Print("UpdateStateMap:");
	upd_map_collision_.Print("DetectCollision:");
	rec_map_.Print("RecoverStateMap:");
	test_stiff_.Print("TestifyStiffness:");
}