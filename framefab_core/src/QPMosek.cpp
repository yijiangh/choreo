#include "QPMosek.h"

#include <iomanip>      // std::setprecision

#define MOSEK_EXISTS

#ifdef MOSEK_EXISTS
#include <mosek.h>
#endif


#define MYOUT std::cout
#define MYERR std::cerr

QPMosek::QPMosek() :QP(), fVal_(std::numeric_limits<double>::infinity())
{
#ifdef MOSEK_EXISTS
	MSKrescodee   r;
	r = MSK_makeenv(&env_, NULL);
	assert(r == MSK_RES_OK);
#endif
	nTasks_ = -1;
	mP_ = 10e-8;
}

QPMosek::~QPMosek()
{
#ifdef MOSEK_EXISTS
	MSK_deleteenv(&env_);
#endif
}

#ifdef MOSEK_EXISTS
/* This function prints log output from MOSEK to the terminal. */
void MSKAPI printstr(void *handle,
	MSKCONST char str[])
{
	(void)handle;
	printf("%s", str);
}
#endif

bool QPMosek::solve(
	const S& H, const V& f,
	const S& C, const V& d,
	V &_x,
	bool _debug)
{
#ifdef MOSEK_EXISTS
	fVal_ = MYINF;
	
	//tSetup.Start();

	bool success = false;

	//number of variables
	MSKint32t numvar = H.rows();
	MSKint32t numcon = d.size();

	MSKint32t     i, j;

	MSKenv_t      env = env_;
	MSKtask_t     task = NULL;
	MSKrescodee   r = MSK_RES_OK;

	if (r == MSK_RES_OK)
	{
		/* Create the optimization task. */
		r = MSK_maketask(env, numcon, numvar, &task);

		if (nTasks_>0)
		{
			MSK_putintparam(task, MSK_IPAR_NUM_THREADS, nTasks_);
		}

		//set precision: see http://docs.mosek.com/7.0/capi/The_optimizers_for_continuous_problems.html#sec-solve-conic
		MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_PFEAS, mP_);		//Controls primal feasibility, default 10e-8
		MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_DFEAS, mP_);		//Controls dual feasibility, default 10e-8
		MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_REL_GAP, mP_);	//Controls relative gap, default 10e-7
		MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_INFEAS, mP_);		//Controls when the problem is declared infeasible, default 10e-10
		MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_MU_RED, mP_);	//Controls when the complementarity is reduced enough, default 10e-8

		//MSK_putintparam(task, MSK_IPAR_PRESOLVE_ELIMINATOR_USE, MSK_OFF);
		MSK_putintparam(task, MSK_IPAR_CHECK_CONVEXITY, MSK_CHECK_CONVEXITY_NONE);

		if (r == MSK_RES_OK)
		{
			if (_debug){ r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr); }

			/* Append 'NUMCON' empty constraints.
			The constraints will initially have no bounds. */
			if (r == MSK_RES_OK)
				r = MSK_appendcons(task, numcon);

			/* Append 'NUMVAR' variables.
			The variables will initially be b_fixed at zero (x=0). */
			if (r == MSK_RES_OK)
				r = MSK_appendvars(task, numvar);

			/* Optionally add a constant term to the objective. */
			if (r == MSK_RES_OK)
				r = MSK_putcfix(task, 0.0);

			/* Copy matrix [A;C] to mosek */
			if (r == MSK_RES_OK)
			{
				if (_debug) { MYOUT << "C: " << std::endl; }

				for (int k = 0; k < C.outerSize() && r == MSK_RES_OK; ++k)
				{
					for (S::InnerIterator it(C, k); it; ++it)
					{
						MSKint32t r = it.row();
						MSKint32t c = it.col();
						double	v = it.value();
						//if (_debug){ MYOUT << "(" << r << "," << c << ") " << /*std::setprecision(16) <<*/ v << std::endl; }
						r = MSK_putaij(task, r, c, v);
					}
				}

				/*
				r = MSK_putacolslice(task,
				0,
				A.cols(),
				A.outerIndexPtr(),
				A.outerIndexPtr()+1,
				A.innerIndexPtr(),
				A.valuePtr());

				r = MSK_putacolslice(task,
				A.cols(),
				A.cols()+C.cols(),
				C.outerIndexPtr(),
				C.outerIndexPtr()+1,
				C.innerIndexPtr(),
				C.valuePtr());
				*/
			}


			for (j = 0; j<numvar && r == MSK_RES_OK; ++j)
			{

				/* Set the linear term c_j in the objective.*/
				if (f.size()>0){
					if (r == MSK_RES_OK)
						r = MSK_putcj(task, j, f[j]);
				}

				/* Set the bounds on variable j.
				blx[j] <= x_j <= bux[j] */
				if (r == MSK_RES_OK){

					r = MSK_putvarbound(task,
						j,           /* Index of variable.*/
						MSK_BK_RA,      /* Bound key.*/
						0,      /* Numerical value of lower bound.*/
						1);     /* Numerical value of upper bound.*/
				}

				//Done before as a whole
				/* Input column j of A */
				//		        if(r == MSK_RES_OK)
				//		          r = MSK_putacol(task,
				//		                          j,                 /* Variable (column) index.*/
				//		                          aptre[j]-aptrb[j], /* Number of non-zeros in column j.*/
				//		                          asub+aptrb[j],     /* Pointer to row indexes of column j.*/
				//		                          aval+aptrb[j]);    /* Pointer to Values of column j.*/

			}

			/* Set the bounds on constraints.
			for i=1, ...,NUMCON : blc[i] <= constraint i <= buc[i] */
			for (i = 0; i<numcon && r == MSK_RES_OK; ++i)
			{
				//double lbi;
				//double ubi;
				//if (!equ){
				//	lbi = -MYINF;
				//	ubi = b[i];
				//}
				//else{
				//	lbi = d[ie];	// beyond b.size() quality constraints
				//	ubi = d[ie];
				//}

				r = MSK_putconbound(task,
					i,				/* Index of constraint.*/
					MSK_BK_FX,		/* Bound key.*/
					d[i],			/* Numerical value of lower bound.*/
					d[i]);			/* Numerical value of upper bound.*/
			}

			if (r == MSK_RES_OK)
			{
				/*
				* The lower triangular part of the Q
				* matrix in the objective is specified.
				*/
				unsigned int nnz = H.nonZeros();
				std::vector<int> hi(nnz + 1), hj(nnz + 1);
				std::vector<double> hv(nnz);


				if (_debug){ MYOUT << "H:" << std::endl; }

				/* Hessian Matrix */
				int c = 0;
				for (int k = 0; k<H.outerSize(); ++k){
					for (S::InnerIterator it(H, k); it; ++it)
					{
						if (it.row() >= it.col())
						{ 
							//only lower triangular part
							hv.at(c) = it.value();
							hi.at(c) = it.row();
							hj.at(c) = it.col();
							//if (_debug){ MYOUT << "(" << it.row() << "," << it.col() << ") " << std::setprecision(16) << it.value() << std::endl; }
							++c;
						}
					}
				}

				/* Input the Q for the objective. */
				r = MSK_putqobj(task, nnz, &hi[0], &hj[0], &hv[0]);

			}

			if (_debug){
				MSK_analyzeproblem(task, MSK_STREAM_MSG);
				MSK_writedata(task, "taskdump.opf");
			}

			//tSetup.Stop();
			
			//tSolve.Start();

			if (r == MSK_RES_OK)
			{
				MSKrescodee trmcode;

				/* Run optimizer */
				r = MSK_optimizetrm(task, &trmcode);

				/* Print a summary containing information
				about the solution for debugging purposes*/
				if (_debug){ MSK_solutionsummary(task, MSK_STREAM_MSG); }

				if (r == MSK_RES_OK)
				{
					MSKsolstae solsta;
					MSK_getsolsta(task, MSK_SOL_ITR, &solsta);
					xFlag_ = solsta;

					switch (solsta)
					{
					case MSK_SOL_STA_OPTIMAL:
					case MSK_SOL_STA_NEAR_OPTIMAL:
						_x.resize(numvar);
						MSK_getxx(task,
							MSK_SOL_ITR,    /* Request the interior solution. */
							_x.data());

						MSK_getprimalobj(task, MSK_SOL_ITR, &fVal_);
						assert(std::isfinite(fVal_));

						success = true;
						//printf("Optimal primal solution\n");
						//for(j=0; j<numvar; ++j)
						//printf("x[%d]: %e\n",j,xx[j]);

						break;
					case MSK_SOL_STA_DUAL_INFEAS_CER:
					case MSK_SOL_STA_PRIM_INFEAS_CER:
					case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
					case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:
						//printf("Primal or dual infeasibility certificate found.\n");
						break;

					case MSK_SOL_STA_UNKNOWN:
						printf("The status of the solution could not be determined.\n");
						break;
					default:
						//printf("Other solution status.");
						break;
					}
				}
				else
				{
					printf("Error while optimizing.\n");
				}
			}

			if (r != MSK_RES_OK)
			{
				/* In case of an error print error code and description. */
				char symname[MSK_MAX_STR_LEN];
				char desc[MSK_MAX_STR_LEN];

				printf("An error occurred while optimizing.\n");
				MSK_getcodedesc(r,
					symname,
					desc);
				printf("Error %s - '%s'\n", symname, desc);

				printf("Qp solver failed while compute CalculateX.\n");
				printf("Press <Enter> to continue.\n");
				getchar();
			}
		}
		MSK_deletetask(&task);
	}

	// tSolve.Stop();

	return success;

#else
	MYOUT << __FUNCTION__ << ": Not compiled with Mosek. Cant optimize." << std::endl;
	return false;
#endif		  
}

std::string QPMosek::report() const
{
	std::stringstream ss;
	ss << "QP using Mosek. exitFlag: " << xFlag_ << " ( " << exitFlagToString(xFlag_) << " ) " << std::endl;
	ss << "Timing: " << tSetup << " setup, " << tSolve << " solve." << std::endl;
	return ss.str();
}

bool QPMosek::solve(const S& H, const V& f, V &_x, const V &x_w, const double & d_tol, bool _debug)
{
	bool success = false;

	//number of variables
	MSKint32t numvar = H.rows();

	//number of constraints (number of nodes), add more than i need
	//refer http://docs.mosek.com/7.1/capi/Efficiency_considerations.html
	MSKint32t numcon = numvar / 6;

	MSKint32t     i, j;

	MSKenv_t      env = env_;
	MSKtask_t     task = NULL;
	MSKrescodee   r = MSK_RES_OK;

	if (r == MSK_RES_OK)
	{
		/* Create the optimization task. */
		r = MSK_maketask(env, numcon, numvar, &task);

		if (nTasks_ > 0)
		{
			MSK_putintparam(task, MSK_IPAR_NUM_THREADS, nTasks_);
		}

		//set precision: see http://docs.mosek.com/7.0/capi/The_optimizers_for_continuous_problems.html#sec-solve-conic
		MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_PFEAS, mP_);		//Controls primal feasibility, default 10e-8
		MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_DFEAS, mP_);		//Controls dual feasibility, default 10e-8
		MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_REL_GAP, mP_);		//Controls relative gap, default 10e-7
		MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_INFEAS, mP_);			//Controls when the problem is declared infeasible, default 10e-10
		MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_MU_RED, mP_);		//Controls when the complementarity is reduced enough, default 10e-8

		//MSK_putintparam(task, MSK_IPAR_PRESOLVE_USE, MSK_PRESOLVE_MODE_OFF);
		//MSK_putintparam(task, MSK_IPAR_PRESOLVE_ELIMINATOR_USE, MSK_OFF);
		MSK_putintparam(task, MSK_IPAR_CHECK_CONVEXITY, MSK_CHECK_CONVEXITY_NONE);

		if (r == MSK_RES_OK)
		{
			if (_debug){ r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr); }

			/* Append 'NUMCON' empty constraints.
			The constraints will initially have no bounds. */
			if (r == MSK_RES_OK)
				r = MSK_appendcons(task, numcon);

			/* Append 'NUMVAR' variables.
			The variables will initially be b_fixed at zero (x=0). */
			if (r == MSK_RES_OK)
				r = MSK_appendvars(task, numvar);

			/* Optionally add a constant term to the objective. */
			if (r == MSK_RES_OK)
				r = MSK_putcfix(task, 0.0);

			for (j = 0; j<numvar && r == MSK_RES_OK; ++j)
			{

				/* Set the linear term c_j in the objective.*/
				if (f.size()>0)
				{
					if (r == MSK_RES_OK)
						r = MSK_putcj(task, j, f[j]);
				}
			}

			double temp_bound = d_tol;
			/* variable bounds */
			for (j = 0; j < numvar/6 && r == MSK_RES_OK; ++j)
			{
				r = MSK_putvarbound(task,
					j * 6,           /* Index of variable.*/
					MSK_BK_RA,      /* Bound key.*/
					-temp_bound,      /* Numerical value of lower bound.*/
					temp_bound);     /* Numerical value of upper bound.*/

				r = MSK_putvarbound(task,
					j * 6 + 1,           /* Index of variable.*/
					MSK_BK_RA,      /* Bound key.*/
					-temp_bound,      /* Numerical value of lower bound.*/
					temp_bound);     /* Numerical value of upper bound.*/
				
				r = MSK_putvarbound(task,
					j * 6 + 2,           /* Index of variable.*/
					MSK_BK_RA,      /* Bound key.*/
					-temp_bound,      /* Numerical value of lower bound.*/
					temp_bound);     /* Numerical value of upper bound.*/

				r = MSK_putvarbound(task,
					j * 6 + 3,           /* Index of variable.*/
					MSK_BK_FR,      /* Bound key.*/
					-MYINF,      /* Numerical value of lower bound.*/
					MYINF);     /* Numerical value of upper bound.*/

				r = MSK_putvarbound(task,
					j * 6 + 4,           /* Index of variable.*/
					MSK_BK_FR,      /* Bound key.*/
					-MYINF,      /* Numerical value of lower bound.*/
					MYINF);     /* Numerical value of upper bound.*/

				r = MSK_putvarbound(task,
					j * 6 + 5,           /* Index of variable.*/
					MSK_BK_FR,      /* Bound key.*/
					-MYINF,      /* Numerical value of lower bound.*/
					MYINF);     /* Numerical value of upper bound.*/
			}

			for (j = 0; j < numvar && r == MSK_RES_OK; ++j)
			{
				/* Set the linear term c_j in the objective.*/
				if (f.size()>0)
				{
					if (r == MSK_RES_OK)
						r = MSK_putcj(task, j, f[j]);
				}
			}

			if (r == MSK_RES_OK)
			{
				/*
				* The lower triangular part of the Q
				* matrix in the objective is specified.
				*/
				unsigned int nnz = H.nonZeros();
				std::vector<int> hi(nnz + 1), hj(nnz + 1);
				std::vector<double> hv(nnz);


				if (_debug){ MYOUT << "H:" << std::endl; }

				/* Hessian Matrix */
				int c = 0;
				for (int k = 0; k<H.outerSize(); ++k){
					for (S::InnerIterator it(H, k); it; ++it)
					{
						if (it.row() >= it.col())
						{
							//only lower triangular part
							hv.at(c) = it.value();
							hi.at(c) = it.row();
							hj.at(c) = it.col();
							//if (_debug){ MYOUT << "(" << it.row() << "," << it.col() << ") " << std::setprecision(16) << it.value() << std::endl; }
							++c;
						}
					}
				}

				/* Input the Q for the objective. */
				r = MSK_putqobj(task, nnz, &hi[0], &hj[0], &hv[0]);
			}

			if (_debug)
			{
				MSK_analyzeproblem(task, MSK_STREAM_MSG);
				MSK_writedata(task, "taskdump.opf");
			}

			if (r == MSK_RES_OK)
			{
				MSKrescodee trmcode;

				/* Run optimizer */
				r = MSK_optimizetrm(task, &trmcode);

				/* Print a summary containing information
				about the solution for debugging purposes*/
				if (_debug){ MSK_solutionsummary(task, MSK_STREAM_MSG); }

				if (r == MSK_RES_OK)
				{
					MSKsolstae solsta;
					MSK_getsolsta(task, MSK_SOL_ITR, &solsta);
					xFlag_ = solsta;

					switch (solsta)
					{
					case MSK_SOL_STA_OPTIMAL:
					case MSK_SOL_STA_NEAR_OPTIMAL:
						_x.resize(numvar);
						MSK_getxx(task,
							MSK_SOL_ITR,    /* Request the interior solution. */
							_x.data());

						MSK_getprimalobj(task, MSK_SOL_ITR, &fVal_);
						assert(std::isfinite(fVal_));

						success = true;
						//printf("Optimal primal solution\n");
						//for(j=0; j<numvar; ++j)
						//printf("x[%d]: %e\n",j,xx[j]);

						break;
					case MSK_SOL_STA_DUAL_INFEAS_CER:
					case MSK_SOL_STA_PRIM_INFEAS_CER:
					case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
					case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:
						//printf("Primal or dual infeasibility certificate found.\n");
						break;

					case MSK_SOL_STA_UNKNOWN:
						printf("The status of the solution could not be determined.\n");
						break;
					default:
						//printf("Other solution status.");
						break;
					}
				}
				else
				{
					printf("Error while optimizing.\n");
				}
			}

			if (r != MSK_RES_OK)
			{
				/* In case of an error print error code and description. */
				char symname[MSK_MAX_STR_LEN];
				char desc[MSK_MAX_STR_LEN];

				printf("An error occurred while optimizing.\n");
				MSK_getcodedesc(r,
					symname,
					desc);
				printf("Error %s - '%s'\n", symname, desc);

				printf("Qp solver failed while compute CalculateD.\n");
				printf("Press <Enter> to continue.\n");
				getchar();
			}
		}
		MSK_deletetask(&task);
	}
	return success;
}

std::string QPMosek::exitFlagToString(int _xflag) const
{
#ifdef MOSEK_EXISTS
	switch (_xflag){
	case MSK_SOL_STA_UNKNOWN:	return "MSK_SOL_STA_UNKNOWN";
	case MSK_SOL_STA_OPTIMAL:	return "MSK_SOL_STA_OPTIMAL";
	case MSK_SOL_STA_PRIM_FEAS:	return "MSK_SOL_STA_PRIM_FEAS";
	case MSK_SOL_STA_DUAL_FEAS:	return "MSK_SOL_STA_DUAL_FEAS";
	case MSK_SOL_STA_PRIM_AND_DUAL_FEAS:	return "MSK_SOL_STA_PRIM_AND_DUAL_FEAS";
	case MSK_SOL_STA_PRIM_INFEAS_CER:	return "MSK_SOL_STA_PRIM_INFEAS_CER";
	case MSK_SOL_STA_DUAL_INFEAS_CER:	return "MSK_SOL_STA_DUAL_INFEAS_CER";
	case MSK_SOL_STA_NEAR_OPTIMAL:	return "MSK_SOL_STA_NEAR_OPTIMAL";
	case MSK_SOL_STA_NEAR_PRIM_FEAS:	return "MSK_SOL_STA_NEAR_PRIM_FEAS";
	case MSK_SOL_STA_NEAR_DUAL_FEAS:	return "MSK_SOL_STA_NEAR_DUAL_FEAS";
	case MSK_SOL_STA_NEAR_PRIM_AND_DUAL_FEAS:	return "MSK_SOL_STA_NEAR_PRIM_AND_DUAL_FEAS";
	case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:	return "MSK_SOL_STA_NEAR_PRIM_INFEAS_CER";
	case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:	return "MSK_SOL_STA_NEAR_DUAL_INFEAS_CER";
	case MSK_SOL_STA_INTEGER_OPTIMAL:	return "MSK_SOL_STA_INTEGER_OPTIMAL";
	case MSK_SOL_STA_NEAR_INTEGER_OPTIMAL:	return "MSK_SOL_STA_NEAR_INTEGER_OPTIMAL";
	}

	return "MSK_SOL_STA_UNKNOWN";
#else
	return std::string();
#endif
}
