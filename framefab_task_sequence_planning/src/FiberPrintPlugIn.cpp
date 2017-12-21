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

	delete ptr_seqanalyzer_;
	ptr_seqanalyzer_ = NULL;

	delete ptr_procanalyzer_;
	ptr_procanalyzer_ = NULL;

	delete ptr_parm_;
	ptr_parm_ = NULL;
}


void FiberPrintPlugIn::Init()
{
	delete ptr_seqanalyzer_;
	ptr_seqanalyzer_ = NULL;

	delete ptr_procanalyzer_;
	ptr_procanalyzer_ = NULL;
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

void FiberPrintPlugIn::GetDeformation()
{
	ptr_dualgraph_->Dualization();

	VX D;
	ptr_stiffness_->Init();
	ptr_stiffness_->CalculateD(D, NULL, false, 0, "FiberTest");
}