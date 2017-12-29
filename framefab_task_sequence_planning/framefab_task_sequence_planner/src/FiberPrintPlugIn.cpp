#include <ros/console.h>

#include "framefab_task_sequence_planner/FiberPrintPlugIn.h"

FiberPrintPlugIn::FiberPrintPlugIn()
{
	ptr_frame_ = NULL;
	ptr_dualgraph_ = NULL;
	ptr_collision_ = NULL;
	ptr_stiffness_ = NULL;

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

	delete ptr_frame_;
	ptr_frame_ = NULL;
}

bool FiberPrintPlugIn::Init()
{
	if(ptr_frame_ != NULL && ptr_parm_ != NULL && ptr_path_ != NULL)
	{
		delete ptr_dualgraph_;
		ptr_dualgraph_ = new DualGraph(ptr_frame_);

		delete ptr_collision_;
		ptr_collision_ = new QuadricCollision(ptr_frame_);

		delete ptr_stiffness_;
		ptr_stiffness_ = new Stiffness(
				ptr_dualgraph_, ptr_parm_,
				ptr_path_, true);

		delete ptr_seqanalyzer_;
		ptr_seqanalyzer_ = NULL;

		delete ptr_procanalyzer_;
		ptr_procanalyzer_ = NULL;

		return true;
	}
	else
	{
		return false;
	}
}

void FiberPrintPlugIn::OneLayerPrint()
{
	fiber_print_.Start();

	if(Init())
	{
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
	else
	{
		ROS_WARN_STREAM("[ts planning] wireframe, fiber_print parm or output_path not initiated."
								<< "ts planning failed.");
	}
}

void FiberPrintPlugIn::GetDeformation()
{
	ptr_dualgraph_->Dualization();

	VX D;
	ptr_stiffness_->Init();
	ptr_stiffness_->CalculateD(D, NULL, false, 0, "FiberTest");
}

bool FiberPrintPlugIn::handleTaskSequencePlanning(
		framefab_msgs::TaskSequencePlanning::Request& req,
		framefab_msgs::TaskSequencePlanning::Response& res)
{
	switch(req.action)
	{
		case framefab_msgs::TaskSequencePlanning::Request::READ_WIREFRAME:
		{
			std::string file_path = req.model_params.file_name;

			// TODO: all of these char* should be const char*
			// convert std::string to writable char*
			std::vector<char> fp(file_path.begin(), file_path.end());
			fp.push_back('\0');

			if(NULL == ptr_frame_)
			{
				ptr_frame_ = new WireFrame();
			}

			// TODO: if contains keyword "pwf"
			ptr_frame_->LoadFromPWF(&fp[0]);

			// TODO add return data for visualization

			break;
		}
		case framefab_msgs::TaskSequencePlanning::Request::TASK_SEQUENCE_SEARCHING:
		{
			double Wl = 1.0;
			double Wp = 1.0;
			double Wa = 1.0;

			if(NULL != ptr_parm_)
			{
				delete ptr_parm_;
			}

			ptr_parm_ = new FiberPrintPARM(Wl, Wp, Wa);
			ptr_path_ = "/home";
			terminal_output_ = true;

			OneLayerPrint();

			//TODO: add return data

			break;
		}
		default:
		{
			ROS_ERROR_STREAM("[ts planning] unknown task sequence planning request action.");
			return false;
		}
	}

	return true;
}
