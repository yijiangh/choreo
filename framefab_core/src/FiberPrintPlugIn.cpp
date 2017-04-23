#include <FiberPrintPlugIn.h>

FiberPrintPlugIn::FiberPrintPlugIn()
{
	ptr_frame_ = NULL;
	ptr_dualgraph_ = NULL;
	ptr_collision_ = NULL;
	ptr_stiffness_ = NULL;

	ptr_graphcut_ = NULL;
	ptr_seqanalyzer_ = NULL;
	ptr_procanalyzer_ = NULL;

	ptr_path_ = NULL;
	ptr_parm_ = NULL;

	terminal_output_ = false;
	file_output_ = false;
}


FiberPrintPlugIn::FiberPrintPlugIn(
	WireFrame *ptr_frame, FiberPrintPARM *ptr_parm, char *ptr_path,
	bool terminal_output, bool file_output
	)
{
	ptr_frame_ = ptr_frame;
	ptr_dualgraph_ = new DualGraph(ptr_frame);
	ptr_collision_ = new QuadricCollision(ptr_frame);
	ptr_stiffness_ = new Stiffness(
		ptr_dualgraph_, ptr_parm, 
		ptr_path, terminal_output
		);

	ptr_graphcut_ = NULL;
	ptr_seqanalyzer_ = NULL;
	ptr_procanalyzer_ = NULL;

	ptr_path_ = ptr_path;
	ptr_parm_ = ptr_parm;

	terminal_output_ = terminal_output;
	file_output_ = file_output;
}


FiberPrintPlugIn::~FiberPrintPlugIn()
{
	delete ptr_dualgraph_;
	ptr_dualgraph_ = NULL;

	delete ptr_collision_;
	ptr_collision_ = NULL;

	delete ptr_stiffness_;
	ptr_stiffness_ = NULL;

	delete ptr_graphcut_;
	ptr_graphcut_ = NULL;

	delete ptr_seqanalyzer_;
	ptr_seqanalyzer_ = NULL;

	delete ptr_procanalyzer_;
	ptr_procanalyzer_ = NULL;

	delete ptr_parm_;
	ptr_parm_ = NULL;
}


void FiberPrintPlugIn::Init()
{
	delete ptr_graphcut_;
	ptr_graphcut_ = NULL;

	delete ptr_seqanalyzer_;
	ptr_seqanalyzer_ = NULL;

	delete ptr_procanalyzer_;
	ptr_procanalyzer_ = NULL;
}


void FiberPrintPlugIn::FrameFabPrint()
{
	fiber_print_.Start();

	Init();

	ptr_graphcut_ = new ADMMCut(
		ptr_dualgraph_,
		ptr_collision_,
		ptr_stiffness_,
		ptr_parm_,
		ptr_path_,
		terminal_output_,
		file_output_
		);

	ptr_seqanalyzer_ = new FFAnalyzer(
		ptr_dualgraph_,
		ptr_collision_,
		ptr_stiffness_,
		ptr_parm_,
		ptr_path_,
		terminal_output_,
		file_output_
		);
	//ptr_procanalyzer_ = new ProcAnalyzer(ptr_seqanalyzer_, ptr_path_);
	
	ptr_graphcut_->MakeLayers();
	cout << "Graph Cut completed." << endl;
	
	if (!ptr_seqanalyzer_->SeqPrint())
	{
		cout << "Model not printable!" << endl;
		getchar();
	}

	printf("FrameFab print done.\n");

	fiber_print_.Stop();
	printf("***Total timer result:\n");
	fiber_print_.Print("FrameFab:");
	ptr_graphcut_->PrintOutTimer();
	ptr_seqanalyzer_->PrintOutTimer();
	ptr_stiffness_->PrintOutTimer();
}


void FiberPrintPlugIn::BruteForcePrint()
{
	fiber_print_.Start();

	Init();

	ptr_seqanalyzer_ = new FFAnalyzer(
		ptr_dualgraph_,
		ptr_collision_,
		ptr_stiffness_,
		ptr_parm_,
		ptr_path_,
		terminal_output_,
		file_output_
		);

	if (!ptr_seqanalyzer_->SeqPrint())
	{
		cout << "Model not printable!" << endl;
		getchar();
	}

	printf("BruteForce print done.\n");

	fiber_print_.Stop();
	printf("***Total timer result:\n");
	fiber_print_.Print("BruteForce:");
	ptr_seqanalyzer_->PrintOutTimer();
	ptr_stiffness_->PrintOutTimer();
}


void FiberPrintPlugIn::SweepingPrint()
{
	Init();

	ptr_graphcut_ = new NormalCut(ptr_frame_, ptr_path_);
	ptr_seqanalyzer_ = new FFAnalyzer(
		ptr_dualgraph_,
		ptr_collision_,
		ptr_stiffness_,
		ptr_parm_,
		ptr_path_,
		terminal_output_,
		file_output_
		);

	ptr_graphcut_->MakeLayers();
	cout << "Graph Cut completed." << endl;

	if (!ptr_seqanalyzer_->SeqPrint())
	{
		cout << "Model not printable!" << endl;
		getchar();

		return;
	}
	printf("Sweeping print done.\n");
}


void FiberPrintPlugIn::OneLayerPrint()
{
	fiber_print_.Start();

	Init();

	ptr_seqanalyzer_ = new FFAnalyzer(
		ptr_dualgraph_,
		ptr_collision_,
		ptr_stiffness_,
		ptr_parm_,
		ptr_path_,
		terminal_output_,
		file_output_
		);

	ptr_procanalyzer_ = new ProcAnalyzer(ptr_seqanalyzer_, ptr_path_);

	if (!ptr_seqanalyzer_->SeqPrint())
	{
		cout << "Model not printable!" << endl;
		getchar();
	}

	fiber_print_.Stop();
	fiber_print_.Print("OneLayer:");

	//ptr_procanalyzer_->ProcPrint();
}

void FiberPrintPlugIn::GetFrameFabCut()
{
	fiber_print_.Start();

	Init();

	ptr_graphcut_ = new ADMMCut(
		ptr_dualgraph_,
		ptr_collision_,
		ptr_stiffness_,
		ptr_parm_,
		ptr_path_,
		terminal_output_,
		file_output_
		);

	ptr_graphcut_->MakeLayers();
	cout << "Graph Cut completed." << endl;

	printf("FrameFab Cut done.\n");

	fiber_print_.Stop();
	printf("***Total timer result:\n");
	fiber_print_.Print("FrameFabCut:");
	ptr_graphcut_->PrintOutTimer();
	ptr_stiffness_->PrintOutTimer();
}

void FiberPrintPlugIn::GetDeformation()
{
	ptr_dualgraph_->Dualization();

	VX D;
	ptr_stiffness_->Init();
	ptr_stiffness_->CalculateD(D, NULL, false, 0, "FiberTest");
}


bool FiberPrintPlugIn::ImportPrintOrder(char *fname)
{ 
	vector<int> queue;

	FILE *fp = fopen(fname, "r");
	int e_id;
	while (fscanf(fp, "%d", &e_id) != EOF)
	{
		queue.push_back(e_id * 2);
	}
	fclose(fp);

	if (queue.size() != ptr_frame_->SizeOfEdgeList() / 2)
	{
		return false;
	}

	if (ptr_seqanalyzer_ == NULL)
	{
		ptr_seqanalyzer_ = new FFAnalyzer(ptr_frame_);
	}
	return ptr_seqanalyzer_->InputPrintOrder(queue);
}


void FiberPrintPlugIn::ExportPrintOrder(char *fname)
{ 
	vector<WF_edge*> queue;
	ptr_seqanalyzer_->OutputPrintOrder(queue);

	int N = queue.size();
	FILE *fp = fopen(fname, "w");
	for (int i = 0; i < N; i++)
	{
		fprintf(fp, "%d\n", queue[i]->ID() / 2);
	}	
	fclose(fp);
}


void FiberPrintPlugIn::Debug()
{
	FILE *fp = fopen("C:/Users/DELL/Desktop/result/Ku.txt", "r");
	int N = ptr_dualgraph_->SizeOfFaceList();
	int Ns = ptr_dualgraph_->SizeOfFreeFace();
	MX tmpK(6 * N, 6 * N);
	tmpK.setZero();
	for (int i = 0; i < 6 * N; i++)
	{
		for (int j = 0; j < 6 * N; j++)
		{
			fscanf(fp, "%lf", &tmpK(i, j));
		}
	}

	MX K(6 * Ns, 6 * Ns);
	K.setZero();
	for (int i = 0; i < N; i++)
	{
		if (!ptr_frame_->isFixed(i))
		{
			int u = ptr_dualgraph_->v_dual_id(i);
			for (int j = 0; j < N; j++)
			{
				if (!ptr_frame_->isFixed(j))
				{
					int v = ptr_dualgraph_->v_dual_id(j);
					for (int h = 0; h < 6; h++)
					{
						for (int l = 0; l < 6; l++)
						{
							K(6 * u + h, 6 * v + l) = tmpK(6 * i + h, 6 * j + l);
						}
					}
				}
			}
		}
	}
	fclose(fp);

	fp = fopen("C:/Users/DELL/Desktop/result/Ku1.txt", "w");
	for (int i = 0; i < 6 * Ns; i++)
	{
		for (int j = 0; j < 6 * Ns; j++)
		{
			fprintf(fp, "%lf ", K(i, j));
		}
		fprintf(fp, "\n");
	}
	fclose(fp);

	fp = fopen("C:/Users/DELL/Desktop/result/Ku2.txt", "w");
	for (int i = 0; i < 6 * Ns; i++)
	{
		for (int j = 0; j < 6 * Ns; j++)
		{
			fprintf(fp, "%lf ", ptr_stiffness_->K_.coeff(i, j));
		}
		fprintf(fp, "\n");
	}
	fclose(fp);
}