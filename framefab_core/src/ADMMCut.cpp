#include "ADMMCut.h"


ADMMCut::ADMMCut()
	:penalty_(0), D_tol_(0), pri_tol_(0), dual_tol_(0)
{
	// This default construction function should never be run
	// We need a mesh to begin with

	ptr_dualgraph_ = NULL;
	ptr_stiffness_ = NULL;
	ptr_collision_ = NULL;
}


ADMMCut::ADMMCut(
	DualGraph			*ptr_dualgraph,
	QuadricCollision	*ptr_collision,
	Stiffness			*ptr_stiffness,
	FiberPrintPARM		*ptr_parm,
	char				*ptr_path,
	bool				terminal_output,
	bool				file_output
	)
{
	ptr_frame_ = ptr_dualgraph->ptr_frame_;
	ptr_dualgraph_ = ptr_dualgraph;
	ptr_stiffness_ = ptr_stiffness;
	ptr_collision_ = ptr_collision;
	ptr_path_ = ptr_path;

	ptr_qp_ = NULL;

	D_tol_ = ptr_parm->ADMM_D_tol_;
	penalty_ = ptr_parm->penalty_;
	pri_tol_ = ptr_parm->pri_tol_;
	dual_tol_ = ptr_parm->dual_tol_;
	
	terminal_output_ = terminal_output;
	file_output_ = file_output;
}


ADMMCut::~ADMMCut()
{
	delete ptr_qp_;
	ptr_qp_ = NULL;
}


void ADMMCut::MakeLayers()
{
	ADMM_cut_.Start();

	cut_round_ = 0;

	// Initial Cutting Edge Setting
	InitState();
	InitWeight();

	vector<double> cut_energy;
	vector<double> res_energy;
	do
	{
		reweight_round_ = 0;

		/* Recreate dual graph at the beginning of each cut */
		SetStartingPoints();
		CreateA();

		/* set x for intial cut setting */
		SetBoundary();

		cut_energy.clear();
		res_energy.clear();

		VX x_prev;
		VX D_prev;
		VX y_prev;
		VX lambda_y_prev;

		do
		{
			ADMM_round_ = 0;

			/* Reweighting loop for cut */
			x_prev = x_;

			do
			{
				if (terminal_output_)
				{
					fprintf(stderr, "---------------------------------------\n");
					fprintf(stderr, "Cut round: %d\nReweight iteration: %d\nADMM round: %d\n",
						cut_round_, reweight_round_, ADMM_round_);
				}

				/*-------------------ADMM loop-------------------*/
				CalculateX();

				/* Update K reweighted by new x */
				ptr_stiffness_->CreateGlobalK(&x_);
				ptr_stiffness_->CreateF(&x_);
				K_ = *(ptr_stiffness_->WeightedK());
				F_ = *(ptr_stiffness_->WeightedF());

				y_prev = y_;
				CalculateY();

				D_prev = D_;
				CalculateD();

				lambda_y_prev = lambda_y_;
				UpdateLambda();

				/*-------------------Residual Calculation-------------------*/
				SpMat Q_new;
				CalculateQ(D_, Q_new);

				//VX dual_res_v		 = -lambda_stf_.transpose() * (Q_ - Q_new);
				//dual_res_v.noalias() += - penalty_ * (y_ - y_prev).transpose() * A_;
				//dual_res_v.noalias() += - (lambda_y_ - lambda_y_prev).transpose() * A_;
				//dual_res_v.noalias() += penalty_ * x_.transpose() * (Q_ - Q_new).transpose() * Q_;

				VX dual_res_v = -lambda_stf_.transpose() * (Q_ - Q_new)
					- penalty_ * (y_ - y_prev).transpose() * A_
					- (lambda_y_ - lambda_y_prev).transpose() * A_
					+ penalty_ * x_.transpose() * (Q_ - Q_new).transpose() * Q_;
				
				dual_res_ = dual_res_v.norm();

				// primal residual has been moved to UpdataLambda()
				// to avoid repeated calculation

				Q_ = Q_new;
				/*-------------------Screenplay-------------------*/

				if (terminal_output_)
				{
					fprintf(stderr, "Dual residual: %lf\nPrimal residual: %lf\n",
						dual_res_, primal_res_);
				}

				ADMM_round_++;
			} while (!TerminationCriteria());

			/* One reweighting process ended! */
			/* Output energy and residual */
			/*-------------------Screenplay-------------------*/
			/* write x distribution to a file */
			if (file_output_)
			{
				double res_tmp = primal_res_;
				res_energy.push_back(res_tmp);

				string str_x = "Cut_" + to_string(cut_round_) + "_Rew_" + to_string(reweight_round_) + "_x";
				Statistics tmp_x(str_x, x_, ptr_path_);
				tmp_x.GenerateVectorFile();

				/* calculate original objective function value */
				double cut_tmp = 0;
				for (int i = 0; i < Md_; i++)
				{
					int dual_u = ptr_dualgraph_->u(i);
					int dual_v = ptr_dualgraph_->v(i);
					int u = ptr_dualgraph_->e_orig_id(dual_u) / 2;
					int v = ptr_dualgraph_->e_orig_id(dual_v) / 2;

					double diffuv;
					double diffvu;

					if (x_[dual_u] - x_[dual_v] > 0)
					{
						diffuv = x_[dual_u] - x_[dual_v];
					}
					else
					{
						diffuv = 0;
					}

					if (x_[dual_v] - x_[dual_u] > 0)
					{
						diffvu = x_[dual_v] - x_[dual_u];
					}
					else
					{
						diffvu = 0;
					}

					cut_tmp += weight_.coeff(u, v) * diffuv;
					cut_tmp += weight_.coeff(v, u) * diffvu;
				}

				cut_energy.push_back(cut_tmp);

				//cout << "Cut " << cut_round_ << " Reweight " << reweight_round_ << " completed." << endl;
				//cout << "Primal Res :" << primal_res_ << endl;
				//cout << "Objective Function :" << cut_tmp << endl;
				//cout << "<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-" << endl;
			}

			reweight_round_++;
		} while (!UpdateR(x_prev));

		if (file_output_)
		{
			string str_eR = "Cut_" + to_string(cut_round_) + "_Res_Energy";
			Statistics s_eR(str_eR, res_energy, ptr_path_);
			s_eR.GenerateStdVecFile();

			string str_eC = "Cut_" + to_string(cut_round_) + "_Cut_Energy";
			Statistics s_eC(str_eC, cut_energy, ptr_path_);
			s_eC.GenerateStdVecFile();
		}

		/* Update New Cut information to Rendering (layer_label_) */

		UpdateCut();

		cut_round_++;

	} while (!CheckLabel());

	ptr_frame_->Unify();

	ADMM_cut_.Stop();
}


void ADMMCut::InitState()
{
	if (terminal_output_)
	{
		init_state_.Start();
	}

	ptr_dualgraph_->Dualization();
	Nd_ = ptr_dualgraph_->SizeOfVertList();
	Md_ = ptr_dualgraph_->SizeOfEdgeList();
	Fd_ = ptr_dualgraph_->SizeOfFaceList();
	Ns_ = ptr_dualgraph_->SizeOfFreeFace();
	N_ = ptr_frame_->SizeOfVertList();
	M_ = ptr_frame_->SizeOfEdgeList();

	// set termination tolerance 
	stop_n_ = floor(N_ / 5);

	ptr_qp_ = QPFactory::make(static_cast<QPFactory::QPType>(1));

	// clear layer label
	for (int i = 0; i < M_; i++)
	{
		ptr_frame_->GetEdge(i)->SetLayer(0);
	}

	// upper dual id 
	for (int i = 0; i < M_; i++)
	{
		WF_edge *e = ptr_frame_->GetEdge(i);
		if (e->isCeiling())
		{
			cutting_edge_.push_back(i);
			//e->SetLayer(1);
			//e->ppair_->SetLayer(1);
		}
	}

	if (terminal_output_)
	{
		init_state_.Stop();
	}
}


void ADMMCut::InitWeight()
{
	if (terminal_output_)
	{
		init_weight_.Start();
	}

	int halfM = M_ / 2;
	weight_.resize(halfM, halfM);
	vector<Triplet<double>> weight_list;
	
	for (int i = 0; i < Md_; i++)
	{
		int orig_u = ptr_dualgraph_->e_orig_id(ptr_dualgraph_->u(i));
		int orig_v = ptr_dualgraph_->e_orig_id(ptr_dualgraph_->v(i));
		WF_edge *e1 = ptr_frame_->GetEdge(orig_u);
		WF_edge *e2 = ptr_frame_->GetEdge(orig_v);
		vector<lld> tmp(3);
		double Fij, Fji;
		double tmp_range;
		double tmp_weight;
		double tmp_height;
			
		tmp_range = ptr_dualgraph_->Weight(i);
		tmp_height = exp(-3 * tmp_range * tmp_range);

		ptr_collision_->DetectCollision(e1, e2, tmp);
		Fji = ptr_collision_->ColFreeAngle(tmp) * 1.0 / ptr_collision_->Divide();

		ptr_collision_->DetectCollision(e2, e1, tmp);
		Fij = ptr_collision_->ColFreeAngle(tmp) * 1.0 / ptr_collision_->Divide();

		tmp_range = max(Fij - Fji, 0.0);
		tmp_weight = exp(-0.5 * tmp_range * tmp_range) * tmp_height;
		if (tmp_weight > SPT_EPS)
		{
			weight_list.push_back(Triplet<double>(orig_u / 2, orig_v / 2, tmp_weight));
		}

		tmp_range = max(Fji - Fij, 0.0);
		tmp_weight = exp(-0.5 * tmp_range * tmp_range) * tmp_height;		
		if (tmp_weight > SPT_EPS)
		{
			weight_list.push_back(Triplet<double>(orig_v / 2, orig_u / 2, tmp_weight));
		}
	}

	weight_.setFromTriplets(weight_list.begin(), weight_list.end());

	if (terminal_output_)
	{
		init_weight_.Stop();
	}
}


void ADMMCut::SetStartingPoints()
{
	if (terminal_output_)
	{
		set_startpoint_.Start();
	}

	if (cut_round_ == 0)
	{
		Nd_w_ = ptr_dualgraph_->SizeOfVertList();
	}
	else
	{
		ptr_dualgraph_->UpdateDualization(&x_);
		Nd_ = ptr_dualgraph_->SizeOfVertList();
		Md_ = ptr_dualgraph_->SizeOfEdgeList();
		Fd_ = ptr_dualgraph_->SizeOfFaceList();
		Ns_ = ptr_dualgraph_->SizeOfFreeFace();
	}

	ptr_stiffness_->Init();

	/* Reweighting Paramter */
	r_.resize(Nd_, Nd_);
	x_.resize(Nd_);
	y_.resize(2 * Md_);
	lambda_stf_.resize(6 * Ns_);
	lambda_y_.resize(2 * Md_);
	a_.resize(Nd_);

	// Set all label x = 1 and calculate KD = F to obtain inital D_0
	r_.setOnes();
	x_.setOnes();
	y_.setZero();
	lambda_stf_.setZero();
	lambda_y_.setZero();
	a_.setZero();

	//ptr_stiffness_->Debug();
	if (terminal_output_)
	{
		set_startpoint_.Stop();
	}
}


void ADMMCut::SetBoundary()
{
	if (terminal_output_)
	{
		set_bound_.Start();
	}

	ptr_stiffness_->CalculateD(D_, &x_);

	// Set lower boundary and upper boundary
	// equality constraints W*x = d
	// Identify Base nodes(1) and Upper Nodes(2)
	// Here we just take the edge with biggest height
	d_.resize(Nd_);
	d_.setZero();

	W_.resize(Nd_, Nd_);
	W_.setZero();

	VectorXi bound(Nd_);
	bound.setZero();

	// upper dual
	int cuts = cutting_edge_.size();
	for (int i = 0; i < cuts; i++)
	{
		int e_id = ptr_dualgraph_->e_dual_id(cutting_edge_[i]);
		bound[e_id] = 2;
		x_[e_id] = 0;

		WF_edge *e = ptr_frame_->GetEdge(cutting_edge_[i]);
		int u = e->pvert_->ID();
		int v = e->ppair_->pvert_->ID();
	}

	// base dual 
	// find boundary nodes, in current stage boundary edge = base edge (for cat_head)
	for (int i = 0; i < M_; i++)
	{
		WF_edge *e = ptr_frame_->GetEdge(i);
		if (e->ID() < e->ppair_->ID())
		{
			int e_id = ptr_dualgraph_->e_dual_id(i);
			int u = e->ppair_->pvert_->ID();
			int v = e->pvert_->ID();
			if (e_id != -1 && (ptr_frame_->isFixed(u) || ptr_frame_->isFixed(v)))
			{
				bound[e_id] = 1;
			}
		}
	}

	vector<Eigen::Triplet<double>> W_list;
	for (int e_id = 0; e_id < Nd_; e_id++)
	{
		if (bound[e_id] == 1)
		{
			d_[e_id] = 1;
			W_list.push_back(Eigen::Triplet<double>(e_id, e_id, 1));
		}
		if (bound[e_id] == 2)
		{
			d_[e_id] = 0;
			W_list.push_back(Eigen::Triplet<double>(e_id, e_id, 1));
		}
	}
	W_.setFromTriplets(W_list.begin(), W_list.end());

	if (terminal_output_)
	{
		set_bound_.Stop();
	}
}

void ADMMCut::CreateA()
{
	if (terminal_output_)
	{
		create_a_.Start();
	}

	A_.resize(2 * Md_, Nd_);
	vector<Triplet<double>> A_list;
	for (int i = 0; i < Md_; i++)
	{
		int dual_u = ptr_dualgraph_->u(i);
		int dual_v = ptr_dualgraph_->v(i);
		A_list.push_back(Triplet<double>(i, dual_u, 1));
		A_list.push_back(Triplet<double>(i, dual_v, -1));
		A_list.push_back(Triplet<double>(i + Md_, dual_v, 1));
		A_list.push_back(Triplet<double>(i + Md_, dual_u, -1));
	}
	
	A_.setFromTriplets(A_list.begin(), A_list.end());

	H1_ = A_.transpose() * A_;

	if (terminal_output_)
	{
		create_a_.Stop();
	}
}

void ADMMCut::CalculateX()
{
	if (terminal_output_)
	{
		cal_x_.Start();
	}

	// Construct Hessian Matrix for X-Qp problem
	if (0 == ADMM_round_)
	{
		CalculateQ(D_, Q_);
	}

	SpMat H2 = Q_.transpose() * Q_;
	SpMat H = penalty_ * (H1_ + H2);

	// Construct Linear coefficient for x-Qp problem
	// Modified @Mar/9/2016, y = Ax constraints came into play.
	a_ = - penalty_ * (A_.transpose() * y_);
	a_.noalias() += A_.transpose() * lambda_y_;
	a_.noalias() += Q_.transpose() * lambda_stf_;

	//a_ = - penalty_ * (A_.transpose() * y_) 
	//	+ A_.transpose() * lambda_y_
	//	+ Q_.transpose() * lambda_stf_;

	// top-down constraints
	// x_i = 0, while strut i belongs to the top
	// x_i = 1, while strut i belongs to the bottom
	if (terminal_output_)
	{
		cal_qp_.Start();
	}

	ptr_qp_->solve(H, a_, W_, d_, x_, false);

	if (terminal_output_)
	{
		cal_qp_.Stop();	
		cal_x_.Stop();
	}
}


void ADMMCut::CalculateQ(const VX _D, SpMat &Q)
{
	if (terminal_output_)
	{
		cal_q_.Start();
	}

	// Construct Hessian Matrix for X-Qp problem
	Q.resize(6 * Ns_, Nd_);
	Q.setZero();
	vector<Eigen::Triplet<double>> Q_list;

	for (int i = 0; i < Ns_; i++)
	{
		int u = ptr_dualgraph_->v_orig_id(i);
		WF_edge *edge = ptr_frame_->GetNeighborEdge(u);

		while (edge != NULL)
		{
			int e_id = ptr_dualgraph_->e_dual_id(edge->ID());
			if (e_id != -1)
			{
				int v = edge->pvert_->ID();
				int j = ptr_dualgraph_->v_dual_id(v);

				MX eKuu = ptr_stiffness_->eKv(edge->ID());
				MX eKeu = ptr_stiffness_->eKe(edge->ID());
				VX Fe = ptr_stiffness_->Fe(edge->ID());
				VX Di(6);
				VX Dj(6);

				if (i < Ns_ && j < Ns_)
				{
					for (int k = 0; k < 6; k++)
					{
						Di[k] = _D[6 * i + k];
						Dj[k] = _D[6 * j + k];
					}
				}
				else
				{
					if (i < Ns_)
					{
						for (int k = 0; k < 6; k++)
						{
							Di[k] = _D[6 * i + k];
							Dj[k] = 0;
						}
					}

					if (j < Ns_)
					{
						for (int k = 0; k < 6; k++)
						{
							Di[k] = 0;
							Dj[k] = _D[6 * j + k];
						}
					}
				}
				VX Gamma = - Fe;
				Gamma.noalias() += eKuu * Di;
				Gamma.noalias() += eKeu * Dj;

				for (int k = 0; k < 6; k++)
				{
					Q_list.push_back(Triplet<double>(6 * i + k, e_id, Gamma[k]));
				}
			}

			edge = edge->pnext_;
		}
	}

	Q.setFromTriplets(Q_list.begin(), Q_list.end());

	if (terminal_output_)
	{
		cal_q_.Stop();
	}
}


void ADMMCut::CalculateD()
{
	if (terminal_output_)
	{
		cal_d_.Start();
	}

	// Ensure that Q is PSD
	SpMat Q = penalty_ * (K_.transpose() * K_);

	if (0 == ADMM_round_)
	{
		K_eps_ = Q.diagonal().sum()/Q.rows() * 0.01;
	}

	Q += (MX::Identity(Q.rows(), Q.rows()) * K_eps_).sparseView();

	VX a = K_.transpose() * lambda_stf_;
	a.noalias() += - penalty_ * (K_.transpose() * F_);

	int Nd = ptr_dualgraph_->SizeOfVertList();
	int Ns = ptr_dualgraph_->SizeOfFreeFace();

	VX D_w(Ns);
	D_w.setZero();

	for (int i = 0; i < Ns; i++)
	{
		WF_edge *e = ptr_frame_->GetEdge(ptr_dualgraph_->e_orig_id(i));
		int u = ptr_dualgraph_->v_dual_id(e->pvert_->ID());
		int v = ptr_dualgraph_->v_dual_id(e->ppair_->pvert_->ID());

		if (u != -1)
		{
			D_w[u] = max(D_w[u], x_[i]);
		}

		if (v != -1)
		{
			D_w[v] = max(D_w[v], x_[i]);
		}
	}

	if (terminal_output_)
	{
		cal_qp_.Start();
	}

	ptr_qp_->solve(Q, a, D_, D_w, D_tol_, false);
	
	if (terminal_output_)
	{
		cal_qp_.Stop();
	}

	if (terminal_output_)
	{
		cal_d_.Stop();
	}
}


void ADMMCut::CalculateY()
{
	if (terminal_output_)
	{
		cal_y_.Start();
	}

	y_.resize(2 * Md_);

	// y_ij = xi - xj (have direction!)
	// seperately minimize each y_ij

	for (int i = 0; i < Md_; i++)
	{
		int dual_u = ptr_dualgraph_->u(i);
		int dual_v = ptr_dualgraph_->v(i);
		int u = ptr_dualgraph_->e_orig_id(dual_u) / 2;
		int v = ptr_dualgraph_->e_orig_id(dual_v) / 2;

		double diffuv = x_[dual_u] - x_[dual_v];
		double diffvu = x_[dual_v] - x_[dual_u];

		double opt_y1, opt_y2;  // optimal y value, y1 = xi - xj, y2 = xj - xi
		double of_c1, of_c2;	// objective function @ optimal y value

		// case #1 y_ij < 0
		// obj func: lamba_ij * y_ij + (mu/2) * (y_ij - (x_i - x_j))^2
		opt_y1 = lambda_y_[i] / penalty_ + diffuv;
		if (opt_y1 < 0)
		{
			of_c1 = (-lambda_y_[i] - penalty_ * diffuv) * opt_y1 + (penalty_ / 2) * pow(opt_y1, 2);
		}
		else
		{
			of_c1 = 0;
		}


		// case #2 y_ij > 0
		// obj func: (1/r_ij) * w_ij^2 * y_ij^2 + lambda_y_ij * y_ij
		// + (penalty/2) * (y_ij - x_i + x_j)^2 
		opt_y2 = (lambda_y_[i] + penalty_ * diffuv) / (2 * pow(weight_.coeff(u, v), 2) / r_(dual_u, dual_v) + penalty_);
		if (opt_y2 > 0)
		{
			of_c2 = (1 / r_(dual_u, dual_v)) * pow(weight_.coeff(u, v) * opt_y2, 2)
				+ (-lambda_y_[i] - penalty_ * diffuv) * opt_y2
				+ (penalty_ / 2) * pow(opt_y2, 2);
		}
		else
		{
			of_c2 = 0;
		}

		if (of_c1 > of_c2)
		{
			y_[i] = opt_y2;
		}
		else
		{
			y_[i] = opt_y1;
		}

		/* ------------------------------------- */

		double r_opt_y1, r_opt_y2;  // optimal y value, y1 = xi - xj, y2 = xj - xi
		double r_of_c1, r_of_c2;	// objective function @ optimal y value

		// case #1 y_ji < 0
		r_opt_y1 = lambda_y_[Md_ + i] / penalty_ + diffvu;
		if (r_opt_y1 < 0)
		{
			r_of_c1 = (-lambda_y_[Md_ + i] - penalty_ * diffvu) * r_opt_y1 + (penalty_ / 2) * pow(r_opt_y1, 2);
		}
		else
		{
			r_of_c1 = 0;
		}


		// case #2 y_ji > 0
		r_opt_y2 = (lambda_y_[Md_ + i] + penalty_ * diffvu) / (2 * pow(weight_.coeff(v, u), 2) / r_(dual_v, dual_u) + penalty_);
		if (r_opt_y2 > 0)
		{
			r_of_c2 = (1 / r_(dual_v, dual_u)) * pow(weight_.coeff(v, u) * r_opt_y2, 2)
				+ (-lambda_y_[Md_ + i] - penalty_ * diffvu) * r_opt_y2
				+ (penalty_ / 2) * pow(r_opt_y2, 2);
		}
		else
		{
			r_of_c2 = 0;
		}

		if (r_of_c1 > r_of_c2)
		{
			y_[Md_ + i] = r_opt_y2;
		}
		else
		{
			y_[Md_ + i] = r_opt_y1;
		}
	}

	if (terminal_output_)
	{
		cal_y_.Stop();
	}
}


void ADMMCut::UpdateLambda()
{
	if (terminal_output_)
	{
		update_lambda_.Start();
	}

	VX tmp_stiff = K_ * D_;
	tmp_stiff.noalias() -= F_;
	
	VX tmp_y = A_ * x_;
	tmp_y.noalias() -= y_;

	lambda_stf_.noalias() += penalty_ * tmp_stiff;

	lambda_y_.noalias()   += penalty_ * tmp_y;

	primal_res_ = (tmp_stiff).norm() + (tmp_y).norm();

	if (terminal_output_)
	{
		update_lambda_.Stop();
	}
}


void ADMMCut::UpdateCut()
{
	if (terminal_output_)
	{
		update_cut_.Start();
	}

	// Convert previous continuous computed result to discrete 1-0 label
	for (int i = 0; i < M_; i++)
	{
		int e_id = ptr_dualgraph_->e_dual_id(i);
		WF_edge *e = ptr_frame_->GetEdge(i);
		int layer = e->Layer();
		if (e_id == -1)
		{
			e->SetLayer(layer + 1);
		}
		else
		{
			if (x_[e_id] >= 0.5)
			{
				x_[e_id] = 1;
			}
			else
			{
				x_[e_id] = 0;
				e->SetLayer(layer + 1);
			}
		}
	}

	// Update cut
	cutting_edge_.clear();
	for (int i = 0; i < Md_; i++)
	{
		int dual_u = ptr_dualgraph_->u(i);
		int dual_v = ptr_dualgraph_->v(i);
		if (x_[dual_u] != x_[dual_v])
		{
			int u = ptr_dualgraph_->e_orig_id(dual_u);
			int v = ptr_dualgraph_->e_orig_id(dual_v);
			if (x_[dual_u] == 1)
			{
				// u is the lower dual vertex of cutting-edge 
				cutting_edge_.push_back(u);
			}
			else
			{
				// v is the lower dual vertex of cutting-edge 
				cutting_edge_.push_back(v);
			}
		}
	}

	if (terminal_output_)
	{
		update_cut_.Stop();
	}
}


bool ADMMCut::UpdateR(VX &x_prev)
{
	if (terminal_output_)
	{
		update_r_.Start();
	}

	double max_improv = 0;
	double int_diff = 0;
	for (int i = 0; i < Nd_; i++)
	{
		/* if No significant improvment found */
		double improv = fabs(x_prev[i] - x_[i]) / (x_prev[i] + 1e-5);
		if (improv > max_improv)
		{
			max_improv = improv;
		}

		//int_diff += min(abs(x_[i] - 0), abs(x_[i] - 1));
	}

	if (terminal_output_)
	{
		fprintf(stderr, "Max relative improvement: %lf\n", max_improv);
	}

	for (int i = 0; i < Md_; i++)
	{
		int dual_u = ptr_dualgraph_->u(i);
		int dual_v = ptr_dualgraph_->v(i);
		int u = ptr_dualgraph_->e_orig_id(dual_u) / 2;
		int v = ptr_dualgraph_->e_orig_id(dual_v) / 2;

		double diffuv;
		double diffvu;

		if (x_[dual_u] - x_[dual_v] > 0)
		{
			diffuv = x_[dual_u] - x_[dual_v];
		}
		else
		{
			diffuv = 0;
		}

		if (x_[dual_v] - x_[dual_u] > 0)
		{
			diffvu = x_[dual_v] - x_[dual_u];
		}
		else
		{
			diffvu = 0;
		}

		r_(dual_u, dual_v) = 1e-5 + weight_.coeff(u, v) * diffuv;
		r_(dual_v, dual_u) = 1e-5 + weight_.coeff(v, u) * diffvu;
	}

	if (terminal_output_)
	{
		update_r_.Stop();
	}

	if (max_improv < 0.8 || reweight_round_ > 20)
	{
		/* Exit Reweighting */
		return true;
	}
	else
	{
		return false;
	}
}


bool ADMMCut::CheckLabel()
{
	int l = 0;													// Number of dual vertex in lower set
	int u = 0;													// Number of dual vertex in upper set

	for (int i = 0; i < Nd_; i++)
	{
		if (x_[i] == 1)
		{
			l++;
		}
		else
		{
			u++;
		}
	}
	
	if (terminal_output_)
	{
		fprintf(stderr, "Lower set edge(whole): %d(%d)\n", l, Nd_w_);
	}

	if (l < 20 || l < ptr_frame_->SizeOfPillar())
	{
		return true;
	}
	else
	{
		return false;
	}

	//return true;
}


bool ADMMCut::TerminationCriteria()
{
	if (ADMM_round_ >= 20)
	{
		return true;
	}

	if (primal_res_ <= pri_tol_ && dual_res_ <= dual_tol_)
	{
		return true;
	}
	else
	{
		if (penalty_ < 1e8 && penalty_ > 0.0001)
		{
			if (primal_res_ > dual_res_)
			{
				penalty_ *= 2;
			}

			if (primal_res_ < dual_res_)
			{
				penalty_ /= 2;
			}
		}
		return false;
	}

	return false;
}


void ADMMCut::PrintOutTimer()
{
	printf("***ADMMCut timer result:\n");
	ADMM_cut_.Print("ADMMCut:");

	if (terminal_output_)
	{
		init_state_.Print("InitState:");
		init_weight_.Print("InitWeight:");
		set_bound_.Print("SetBoundary:");
		set_startpoint_.Print("SetStartPoint:");
		create_a_.Print("CreateA:");
		cal_x_.Print("CalculateX:");
		cal_y_.Print("CalculateY:");
		cal_q_.Print("CalculateQ:");
		cal_d_.Print("CalculateD:");
		cal_qp_.Print("qp:");
		update_lambda_.Print("UpdateLambda:");
		update_cut_.Print("UpdateCut:");
		update_r_.Print("UpdateR:");
	}
	else
	{
		printf("***ADMMCut detailed timing turned off.\n");
	}
}


void ADMMCut::WriteWeight()
{
	//string path = path_;

	//string weight_path = path + "/point_weight.txt";
	//string line_path = path + "/line.txt";

	//FILE *fp = fopen(weight_path.c_str(), "w+");

	//int N = ptr_frame_->SizeOfVertList();
	//double minz = ptr_dualgraph_->minZ();
	//double maxz = ptr_dualgraph_->maxZ();
	//vector<double> ww(N);
	//for (int i = 0; i < N; i++)
	//{
	//	if (ptr_dualgraph_->isExistingVert(i))
	//	{
	//		if (ptr_frame_->GetDegree(i) > 1)
	//		{
	//			//double w = 1 - (verts[i]->Position().z() - minz) / (maxz - minz);
	//			double w = exp(-3 * pow((ptr_frame_->GetPosition(i).z() - minz) / (maxz - minz), 2));
	//			ww[i] = w;
	//		}
	//		else
	//		{
	//			ww[i] = 1.0;
	//		}
	//	}
	//}

	//for (int i = 0; i < N; i++)
	//{
	//	point p = ptr_frame_->GetVert(i)->RenderPos();
	//	fprintf(fp, "%lf %lf %lf ", p.x(), p.y(), p.z());

	//	double r;
	//	double g;
	//	double b;

	//	if (ww[i] < 0.25)
	//	{
	//		r = 0.0;
	//		g = ww[i] * 4.0;
	//		b = 1.0;
	//	}
	//	else
	//		if (ww[i] < 0.5)
	//		{
	//			r = 0.0;
	//			g = 1.0;
	//			b = (0.5 - ww[i]) * 4.0;
	//		}
	//		else
	//			if (ww[i] < 0.75)
	//			{
	//				r = (ww[i] - 0.5) * 4.0;
	//				g = 1.0;
	//				b = 0.0;
	//			}
	//			else
	//			{
	//				r = 1.0;
	//				g = (1.0 - ww[i]) * 4.0;
	//				b = 0.0;
	//			}

	//	fprintf(fp, "%lf %lf %lf\n", r, g, b);
	//}

	//fclose(fp);
	////ptr_frame_->ExportLines(line_path.c_str());
}


void ADMMCut::WriteStiffness()
{
	char l[5];
	sprintf(l, "%d", cut_round_);
	string path = ptr_path_;
	string offset_path = path + "/stiffness_" + l + ".txt";

	FILE *fp = fopen(offset_path.c_str(), "w+");
	fprintf(fp, "#offset colormap#\r\n");

	int N = ptr_frame_->SizeOfVertList();
	vector<double> ss(N);
	for (int i = 0; i < N; i++)
	{
		if (ptr_dualgraph_->isExistingVert(i) && !ptr_frame_->isFixed(i))
		{
			int j = ptr_dualgraph_->v_dual_id(i);

			VX offset(3);
			for (int k = 0; k < 3; k++)
			{
				offset[k] = D_[j * 6 + k];
			}

			if (offset.norm() >= D_tol_)
			{
				printf(".............. %lf\n", offset.norm());
				getchar();
			}
			ss[i] = offset.norm() / D_tol_;
		}
		else
		{
			ss[i] = 0.0;
		}

		//if (ptr_dualgraph_->isExistingVert(i))
		{
			point p = ptr_frame_->GetVert(i)->RenderPos();
			fprintf(fp, "%lf %lf %lf ", p.x(), p.y(), p.z());

			double r;
			double g;
			double b;

			if (ss[i] < 0.25)
			{
				r = 0.0;
				g = ss[i] * 4.0;
				b = 1.0;
			}
			else
				if (ss[i] < 0.5)
				{
					r = 0.0;
					g = 1.0;
					b = (0.5 - ss[i]) * 4.0;
				}
				else
					if (ss[i] < 0.75)
					{
						r = (ss[i] - 0.5) * 4.0;
						g = 1.0;
						b = 0.0;
					}
					else
					{
						r = 1.0;
						g = (1.0 - ss[i]) * 4.0;
						b = 0.0;
					}

			fprintf(fp, "%lf %lf %lf\r\n", r, g, b);
		}
	}

	fclose(fp);
}


void ADMMCut::Debug()
{
}