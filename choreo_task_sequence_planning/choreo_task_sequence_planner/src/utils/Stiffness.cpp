#include "choreo_task_sequence_planner/utils/Stiffness.h"

Stiffness::Stiffness()
{
	terminal_output_ = false;
	file_output_ = false;
}


Stiffness::Stiffness(DualGraph *ptr_dualgraph)
	:r_(0.6), nr_(0), density_(1210 * 1e-12), g_(-9806.33), G_(1032), E_(1100), v_(0.39), shear_(0)
{
	ptr_dualgraph_ = ptr_dualgraph;

	Init();

	terminal_output_ = false;
	file_output_ = false;
}


Stiffness::Stiffness(
	DualGraph *ptr_dualgraph, FiberPrintPARM *ptr_parm, char *ptr_path,
	bool terminal_output, bool file_output
	)
{
	/* in use */
	ptr_dualgraph_ = ptr_dualgraph;
	ptr_parm_ = ptr_parm;
	ptr_path_ = ptr_path;

	r_ = ptr_parm->radius_;
	nr_ = 0.0;
	density_ = ptr_parm->density_;
	g_ = ptr_parm->g_;
	G_ = ptr_parm->shear_modulus_;
	E_ = ptr_parm->youngs_modulus_;
	v_ = ptr_parm->poisson_ratio_;

	shear_ = 0;

	Init(); 

	terminal_output_ = terminal_output;
	file_output_ = file_output;
}


Stiffness::~Stiffness()
{
}


void Stiffness::Init()
{
	CreateFe();
	CreateElasticK();

    Ns_ = ptr_dualgraph_->SizeOfFreeFace();
}


void Stiffness::CreateFe()
{
	if (terminal_output_)
	{
		create_fe_.Start();
	}

	WireFrame *ptr_frame = ptr_dualgraph_->ptr_frame_;
	int Nd = ptr_dualgraph_->SizeOfVertList();
	int Fd = ptr_dualgraph_->SizeOfFaceList();

	CoordTrans coord_trans;
	double Ax = M_PI * r_ * r_;
	double gx = 0;
	double gy = 0;
	double gz = g_;

	Fe_.resize(Nd);
	for (int i = 0; i < Nd; i++)
	{
		Fe_[i].setZero();

		WF_edge *ei = ptr_frame->GetEdge(ptr_dualgraph_->e_orig_id(i));
		WF_edge *ej = ei->ppair_;
		int dual_u = ptr_dualgraph_->v_dual_id(ej->pvert_->ID());
		int dual_v = ptr_dualgraph_->v_dual_id(ei->pvert_->ID());

		double t0, t1, t2, t3, t4, t5, t6, t7, t8;
		coord_trans.CreateTransMatrix(ej->pvert_->Position(), ei->pvert_->Position(),
			t0, t1, t2, t3, t4, t5, t6, t7, t8, 0.0);

		VectorXd Fei(12);
		Fei.setZero();

		double L = ei->Length();
		Fei[0] = Fei[6] = density_ * Ax * L * gx / 2.0;
		Fei[1] = Fei[7] = density_ * Ax * L * gy / 2.0;
		Fei[2] = Fei[8] = density_ * Ax * L * gz / 2.0;
		
		Fei[3] = density_ * Ax * L * L / 12.0 * ((-t3*t7 + t4*t6)*gy + (-t3*t8 + t5*t6)*gz);
		Fei[4] = density_ * Ax * L * L / 12.0 * ((-t4*t6 + t3*t7)*gx + (-t4*t8 + t5*t7)*gz);
		Fei[5] = density_ * Ax * L * L / 12.0 * ((-t5*t6 + t3*t8)*gx + (-t5*t7 + t4*t8)*gy);
		
		Fei[9] = density_ * Ax * L * L / 12.0 * ((t3*t7 - t4*t6)*gy + (t3*t8 - t5*t6)*gz);
		Fei[10] = density_ * Ax * L * L / 12.0 * ((t4*t6 - t3*t7)*gx + (t4*t8 - t5*t7)*gz);
		Fei[11] = density_ * Ax * L * L / 12.0 * ((t5*t6 - t3*t8)*gx + (t5*t7 - t4*t8)*gy);

		Fe_[i] = Fei;
	}

	if (terminal_output_)
	{
		create_fe_.Stop();
	}
}


void Stiffness::CreateF(VX *ptr_x)
{
	if (terminal_output_)
	{
		create_f_.Start();
	}

	/* Run only after CreadFe is done! */
	WireFrame *ptr_frame = ptr_dualgraph_->ptr_frame_;
	int Nd = ptr_dualgraph_->SizeOfVertList();
	int Fd = ptr_dualgraph_->SizeOfFaceList();

	VX x;
	if (ptr_x == NULL)
	{
		ptr_x = &x;
		ptr_x->resize(Nd);
		ptr_x->setOnes();
	}

	F_.resize(6 * Ns_);
	F_.setZero();

	for (int i = 0; i < Nd; i++)
	{
		WF_edge *ei = ptr_frame->GetEdge(ptr_dualgraph_->e_orig_id(i));
		WF_edge *ej = ei->ppair_;
		int dual_u = ptr_dualgraph_->v_dual_id(ej->pvert_->ID());
		int dual_v = ptr_dualgraph_->v_dual_id(ei->pvert_->ID());

		for (int j = 0; j < 6; j++)
		{
            // only unrestrained node is added into stiffness equation
            if (dual_u < Ns_)
            {
                F_[dual_u * 6 + j] += (*ptr_x)[i] * Fe_[i][j];
            }
            if (dual_v < Ns_)
            {
				F_[dual_v * 6 + j] += (*ptr_x)[i] * Fe_[i][j + 6];
            }
		}
	}

	if (terminal_output_)
	{
		create_f_.Stop();
	}
}


void Stiffness::CreateElasticK()
{
	if (terminal_output_)
	{
		create_ek_.Start();
	}

	WireFrame		  *ptr_frame   = ptr_dualgraph_->ptr_frame_;
	vector<WF_edge *> wf_edge_list = *ptr_frame->GetEdgeList();
	vector<WF_vert *> wf_vert_list = *ptr_frame->GetVertList();

	int Nd = ptr_dualgraph_->SizeOfVertList();
	int Fd = ptr_dualgraph_->SizeOfFaceList();

	/* ref to Matrix Analysis of Strutures Aslam Kassimali, Section 8.2 Table 8.1*/
	double Ax = F_PI * r_ * r_;
	double Asy = Ax * (6 + 12 * v_ + 6 * v_*v_) / (7 + 12 * v_ + 4 * v_*v_);
	double Asz = Asy;

	/* torsion constant */
	double Jxx = 0.5 * F_PI * r_ * r_ * r_ * r_;
	
	/* shear deformation constant */
	double Ksy = 0;
	double Ksz = 0;

	/* area moment of inertia (bending about local y,z-axis)
	* https://en.wikipedia.org/wiki/Bending (beam deflection equation)
	* https://en.wikipedia.org/wiki/List_of_area_moments_of_inertia (numerical value)
	* note this is slender rod of length L and Mass M, spinning around end
	*/
	double Iyy = F_PI * r_ * r_ * r_ * r_ / 4;
	double Izz = Iyy;

	double   t0, t1, t2, t3, t4, t5, t6, t7, t8;     /* coord transf matrix entries */

	eK_.resize(Nd);
	for (int i = 0; i < Nd; i++)
	{
		eK_[i].setZero();

		//WF_edge *ei = ptr_frame->GetNeighborEdge(ptr_dualgraph_->v_orig_id(i));
		WF_edge *ei = wf_edge_list[ptr_dualgraph_->e_orig_id(i)];

		int u = ei->ppair_->pvert_->ID();
		int v = ei->pvert_->ID();
		double L = ei->Length();
		double Le = L - 2 * nr_;

		MatrixXd eKuv(12, 12);
		eKuv.setZero();

		point node_u = wf_vert_list[u]->Position();
		point node_v = wf_vert_list[v]->Position();

		transf_.CreateTransMatrix(node_u, node_v, t0, t1, t2, t3, t4, t5, t6, t7, t8, 0);
		
		if (1 == shear_)
		{
			/* for circular cross-sections, the shape factor for shear(fs) = 1.2 (ref. Aslam's book) */
			double fs = 1.2;
			Ksy = 12. * E_ * Izz * fs / (G_ * Asy * Le * Le);
			Ksz = 12. * E_ * Iyy * fs / (G_ * Asz * Le * Le);
		}
		else
		{
			Ksy = 0;
			Ksz = 0;
		}

        int n1 = ptr_dualgraph_->v_dual_id(u);
        int n2 = ptr_dualgraph_->v_dual_id(v);

		eKuv(0,0) = eKuv(6,6) = E_ * Ax / Le;
		eKuv(1,1) = eKuv(7,7) = 12. * E_ * Izz / (Le * Le * Le * (1. + Ksy));
		eKuv(2,2) = eKuv(8,8) = 12. * E_ * Iyy / (Le * Le * Le * (1. + Ksz));
		eKuv(3,3) = eKuv(9,9) =  G_ * Jxx / Le;
		eKuv(4,4) = eKuv(10,10) = (4. + Ksz) * E_ * Iyy / (Le * (1. + Ksz));
		eKuv(5,5) = eKuv(11,11) = (4. + Ksy) * E_ * Izz / (Le * (1. + Ksy));

		eKuv(4,2) = eKuv(2,4) = -6. * E_ * Iyy / (Le * Le * (1. + Ksz));
		eKuv(5,1) = eKuv(1,5) = 6. * E_ * Izz / (Le * Le * (1. + Ksy));
		eKuv(6,0) = eKuv(0,6) = - eKuv(0,0);

		eKuv(11,7) = eKuv(7,11) = eKuv(7,5) = eKuv(5,7) = -eKuv(5,1);
		eKuv(10,8) = eKuv(8,10) = eKuv(8,4) = eKuv(4,8) = -eKuv(4,2);
		eKuv(9,3) = eKuv(3,9)   = - eKuv(3,3);
		eKuv(10, 2) = eKuv(2, 10) = eKuv(4, 2);
		eKuv(11, 1) = eKuv(1, 11) = eKuv(5, 1);

		eKuv(7,1) = eKuv(1,7) = -eKuv(1,1);
		eKuv(8,2) = eKuv(2,8) = -eKuv(2,2);
		eKuv(10,4) = eKuv(4,10) = (2. - Ksz) * E_ * Iyy / (Le * (1. + Ksz));
		eKuv(11,5) = eKuv(5,11) = (2. - Ksy) * E_ * Izz / (Le * (1. + Ksy));

		transf_.TransLocToGlob(t0, t1, t2, t3, t4, t5, t6, t7, t8, eKuv, 0, 0);

		for (int k = 0; k < 12; k++)
		{
			for (int l = k + 1; l < 12; l++)
			{
				if (eKuv(k, l) != eKuv(l, k))
				{
					if (fabs(eKuv(k,l) / eKuv(l,k) - 1.0) > SPT_EPS
						&& 
						(fabs(eKuv(k, l) / eKuv(k, k)) > SPT_EPS || fabs(eKuv(l, k) / eKuv(k, k)) > SPT_EPS)
						)
					{
						fprintf(stderr, "elastic_K: element stiffness matrix not symetric ...\n");
						fprintf(stderr, " ... k[%d][%d] = %15.6e \n", k, l, eKuv(k,l));
						fprintf(stderr, " ... k[%d][%d] = %15.6e   ", l, k, eKuv(l,k));
						fprintf(stderr, " ... relative error = %e \n", fabs(eKuv(k,l) / eKuv(l,k) - 1.0));
					}

					eKuv(k, l) = eKuv(l, k) = (eKuv(k, l) + eKuv(l, k)) / 2;
				}
			}
		}

		eK_[i] = eKuv;
	}

	if (terminal_output_)
	{
		create_ek_.Stop();
	}
}


void Stiffness::CreateGlobalK(VX *ptr_x)
{
	if (terminal_output_)
	{
		create_k_.Start();
	}

	WireFrame *ptr_frame = ptr_dualgraph_->ptr_frame_;
	int Nd = ptr_dualgraph_->SizeOfVertList();
	int Fd = ptr_dualgraph_->SizeOfFaceList();

	VX x;
	if (ptr_x == NULL)
	{
		ptr_x = &x;
		ptr_x->resize(Nd);
		ptr_x->setOnes();
	}

	vector<Triplet<double>> K_list;

	K_.resize(6 * Ns_, 6 * Ns_);
	for (int i = 0; i < Nd; i++)
	{
		WF_edge *ei = ptr_frame->GetEdge(ptr_dualgraph_->e_orig_id(i));
		WF_edge *ej = ei->ppair_;
		int u = ej->pvert_->ID();
		int v = ei->pvert_->ID();
		int dual_u = ptr_dualgraph_->v_dual_id(u);
		int dual_v = ptr_dualgraph_->v_dual_id(v);

		if (dual_u < Ns_ && dual_v < Ns_)
		{
            // dual_u and dual_v are both unrestrained node
			for (int k = 0; k < 6; k++)
			{
				for (int l = 0; l < 6; l++)
				{
					double tmp;

					tmp = (*ptr_x)[i] * eK_[i](k, l);
					if (fabs(tmp) > SPT_EPS)
					{
						K_list.push_back(Triplet<double>(dual_u * 6 + k, dual_u * 6 + l, tmp));
					}

					tmp = (*ptr_x)[i] * eK_[i](k + 6, l + 6);
					if (fabs(tmp) > SPT_EPS)
					{
						K_list.push_back(Triplet<double>(dual_v * 6 + k, dual_v * 6 + l, tmp));
					}

					tmp = (*ptr_x)[i] * eK_[i](k, l + 6);
					if (fabs(tmp) > SPT_EPS)
					{
						K_list.push_back(Triplet<double>(dual_u * 6 + k, dual_v * 6 + l, tmp));
					}

					tmp = (*ptr_x)[i] * eK_[i](k + 6, l);
					if (fabs(tmp) > SPT_EPS)
					{
						K_list.push_back(Triplet<double>(dual_v * 6 + k, dual_u * 6 + l, tmp));
					}
				}
			}
		}
		else
		if (dual_u < Ns_)
		{
            // dual_u is free while dual_v is restrained
			for (int k = 0; k < 6; k++)
			{
				for (int l = 0; l < 6; l++)
				{
					double tmp = (*ptr_x)[i] * eK_[i](k, l);
					if (fabs(tmp) > SPT_EPS)
					{
						K_list.push_back(Triplet<double>(dual_u * 6 + k, dual_u * 6 + l, tmp));
					}
				}
			}
		}
		else
		if (dual_v < Ns_)
		{
			for (int k = 0; k < 6; k++)
			{
				for (int l = 0; l < 6; l++)
				{
					double tmp = (*ptr_x)[i] * eK_[i](k + 6, l + 6);
					if (fabs(tmp) > SPT_EPS)
					{
						K_list.push_back(Triplet<double>(dual_v * 6 + k, dual_v * 6 + l, tmp));
					}
				}
			}
		}
	}
	K_.setFromTriplets(K_list.begin(), K_list.end());

	if (terminal_output_)
	{
		create_k_.Stop();
	}
}


bool Stiffness::CalculateD(
	VX &D, VX *ptr_x, 
	bool cond_num, int file_id, string file_name
	)
{
	D.resize(6 * Ns_);
	D.setZero();

	CreateGlobalK(ptr_x);
	CreateF(ptr_x);

	/* Parameter for StiffnessSolver */
	int	info;
	IllCondDetector	stiff_inspector(K_);

	/* --- Check Stiffness Matrix Condition Number --- */
	if (cond_num)
	{
		if (!CheckIllCondition(stiff_inspector))
		{
			return false;
		}
	}

	/* --- Solving Process --- */
	if (terminal_output_)
	{
		fprintf(stdout, "Stiffness : Linear Elastic Analysis ... Element Gravity Loads\n");
	}
	
	if (!stiff_solver_.SolveSystem(K_, D, F_, terminal_output_, info))
	{
		cout << "Stiffness Solver fail!\n" << endl;
		return false;
	}

	/* --- Equilibrium Error Check --- */
	if (!CheckError(stiff_inspector, D))
	{
		return false;
	}

	/* --- Output Process --- */
	if (file_output_)
	{
		WriteData(D, file_id, file_name);
	}

	return true;
}


bool Stiffness::CalculateD(
	VX &D, VX &D0, VX *ptr_x,
	bool cond_num,
	int file_id, string file_name
	)
{
	D.resize(6 * Ns_);
	D.setZero();

	CreateGlobalK(ptr_x);
	CreateF(ptr_x);

	/* --- Check Stiffness Matrix Condition Number --- */
	IllCondDetector	stiff_inspector(K_);
	if (cond_num)
	{
		if (!CheckIllCondition(stiff_inspector))
		{
			return false;
		}
	}

	/* --- Solving Process --- */
	if (terminal_output_)
	{
		fprintf(stdout, "Stiffness : Linear Elastic Analysis ... Element Gravity Loads\n");
	}

	int	info;
	if (!stiff_solver_.SolveSystem(K_, D, F_, D0, terminal_output_, info))
	{
		cout << "Stiffness Solver fail!\n" << endl;
		return false;
	}

	/* --- Equilibrium Error Check --- */
	if (!CheckError(stiff_inspector, D))
	{
		return false;
	}

	/* --- Output Process --- */
	if (file_output_)
	{
		WriteData(D, file_id, file_name);
	}

	return true;
}


bool Stiffness::CheckIllCondition(IllCondDetector &stiff_inspector)
{	
	if (terminal_output_)
	{
		check_ill_.Start();
	}

	bool bSuccess = true;
	double cond_num;
	cond_num = stiff_inspector.ComputeCondNum();
	printf("Condition Number = %9.3e\n", cond_num);
	if (cond_num < MCOND_TOL)
	{
		if (terminal_output_)
		{
			printf(" < tol = %7.1e\n", MCOND_TOL);
			printf(" * Acceptable Matrix! *\n");
		}
	}
	else
	{
		printf(" > tol = %7.1e\n", MCOND_TOL);
		printf(" * Ill Conditioned Stiffness Matrix! *\n");
		printf("Press any key to exit...\n");
		bSuccess = false;
	}

	if (terminal_output_)
	{
		check_ill_.Stop();
	}

	return bSuccess;
}


bool Stiffness::CheckError(IllCondDetector &stiff_inspector, VX &D)
{
	if (terminal_output_)
	{
		check_error_.Start();
	}

	bool bSuccess = true;
	double error = stiff_inspector.EquilibriumError(K_, D, F_);
	if (terminal_output_)
	{
		printf("Root Mean Square (RMS) equilibrium error = %9.3e\n", error);
	}
	if (error < STIFF_TOL)
	{
		if (terminal_output_)
		{
			printf(" < tol = %7.1e\n", STIFF_TOL);
			printf(" * Converged *\n");
		}
	}
	else
	{
		printf(" > tol = %7.1e\n", STIFF_TOL);
		printf(" !! Not Converged !!\n");
		printf("Press any key to exit...\n");
		bSuccess = false;
	}

	if (terminal_output_)
	{
		check_error_.Stop();
	}

	return bSuccess;
}


void Stiffness::WriteData(VectorXd &D, int id, string fname)
{	
	if (fname == "")
	{
		return;
	}

	VX D_joined(ptr_dualgraph_->SizeOfFaceList() * 6);
	D_joined.setZero();

	for (int i = 0; i < Ns_ * 6; i++)
	{
		D_joined[i] = D[i];
	}


	/* --- Gnuplot File Generation --- */
	char iname[10];
	sprintf(iname, "%d", id);

	string path = ptr_path_;
	string fpath = path + '/' + fname + iname + ".3dd";
	string meshpath = path + '/' + fname + iname + "-msh";
	string plotpath = path + '/' + fname + iname + ".plt";

	double  exagg_static = 5;
	float	scale = 1;

	stiff_io_.WriteInputData(fpath.c_str(), ptr_dualgraph_, ptr_parm_, terminal_output_);
	stiff_io_.GnuPltStaticMesh(fpath.c_str(), meshpath.c_str(), plotpath.c_str(),
		D_joined, exagg_static, scale, ptr_dualgraph_, ptr_dualgraph_->ptr_frame_);
}


MatrixXd Stiffness::eKe(int ei)
{
	MX tmpK(6, 6);
	tmpK.setZero();

	int dual_id = ptr_dualgraph_->e_dual_id(ei);
	if (ptr_dualgraph_->e_orig_id(dual_id) == ei)
	{
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				tmpK(i, j) = eK_[dual_id](i, j + 6);
			}
		}
	}
	else
	{
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				tmpK(i, j) = eK_[dual_id](i + 6, j);
			}
		}
	}

	return tmpK;
}


MatrixXd Stiffness::eKv(int ei)
{
	MX tmpK(6, 6);
	tmpK.setZero();

	int dual_id = ptr_dualgraph_->e_dual_id(ei);
	if (ptr_dualgraph_->e_orig_id(dual_id) == ei)
	{
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				tmpK(i, j) = eK_[dual_id](i, j);
			}
		}
	}
	else
	{
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				tmpK(i, j) = eK_[dual_id](i + 6, j + 6);
			}
		}
	}

	return tmpK;
}


VectorXd Stiffness::Fe(int ei)
{
	VX tmpF(6);
	tmpF.setZero();

	int dual_id = ptr_dualgraph_->e_dual_id(ei);
	if (ptr_dualgraph_->e_orig_id(dual_id) == ei)
	{
		for (int j = 0; j < 6; j++)
		{
			tmpF[j] = Fe_[dual_id][j];
		}
	}
	else
	{
		for (int j = 0; j < 6; j++)
		{
			tmpF[j] = Fe_[dual_id][j + 6];
		}
	}
	return tmpF;
}


void Stiffness::PrintOutTimer()
{
	if (terminal_output_)
	{
		printf("***Stiffness timer result:\n");
		stiff_solver_.compute_k_.Print("ComputeK:");
		stiff_solver_.solve_d_.Print("SolveD:");
		create_fe_.Print("CreateFe:");
		create_f_.Print("CreateF:");
		create_ek_.Print("CreateElasticK:");
		create_k_.Print("CreateGlobalK:");
		check_ill_.Print("CheckIllCond:");
		check_error_.Print("CheckError:");
	}
	else
	{
		printf("***Stiffness detailed timing turned off.\n");
	}
}