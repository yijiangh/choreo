#include "choreo_task_sequence_planner/utils/StiffnessIO.h"

/*
* OUTPUT_PATH
* return path for output files using specific output directory
*/
void StiffnessIO::OutputPath(const char *fname, char fullpath[], const int len, char *default_outdir, int verbose) 
{
	int res;
	assert(fname != NULL);

	const char *outdir;
	outdir = getenv("FRAMEFAB_OUTDIR");
	if (outdir == NULL) 
	{
		if (default_outdir == NULL)
			outdir = TempDir();
		else
			outdir = default_outdir;
	}

	res = sprintf(fullpath, "%s%c%s", outdir, '\\', fname);

	if (res > len) 
	{
		printf("ERROR: unable to construct output filename: overflow.\n");
		exit(16);
	}

	if (verbose)
	{
		printf("StiffnessIO->OuputPath: Output file path generated: %s\n", fullpath); /* debug */
	}
}


/*
* GnuPltStaticMesh  - create mesh data of deformed and undeformed mesh  25/Nov/2015
* use gnuplot
* useful gnuplot options: unset xtics ytics ztics border view key
* This function illustrates how to read the internal force output data file.
* The internal force output data file contains all the information required
* to plot deformed meshes, internal axial force, internal shear force, internal
* torsion, and internal bending moment diagrams.
*/
void StiffnessIO::GnuPltStaticMesh(
	const char *fpath,
	const char *meshpath, const char *plotpath,
	VX &D,
	double exagg_static, float scale,
	DualGraph *ptr_dualgraph, WireFrame *ptr_frame
	)
{
	int nN = ptr_dualgraph->SizeOfFaceList();
	int nE = ptr_dualgraph->SizeOfVertList();
	WireFrame *ptr_wf = ptr_dualgraph->ptr_frame_;
	std::vector<WF_edge*> wf_edge_list = *ptr_wf->GetEdgeList();
	std::vector<DualFace*> dual_face_list = *ptr_dualgraph->GetFaceList();

	int anlyz = 1;

	FILE	*fpif = NULL, *fpm = NULL;
	double	mx, my, mz; /* coordinates of the frame element number labels */
	char	fnif[FILENMAX], meshfl[FILENMAX],
		D3 = ' ',
		errMsg[MAXL],
		ch = 'a';
	int	sfrv = 0,		/* *scanf return value			*/
		frel, nx,	/* frame element number, number of increments */
		n1, n2;		/* node numbers			*/
	float	x1, y1, z1,	/* coordinates of node n1		*/
		x2, y2, z2;	/* coordinates of node n2		*/
	int	j = 0, m = 0, n = 0,
		X = 0, Y = 0, Z = 0,
		lw = 1;		/*  line width of deformed mesh		*/
	time_t  now;		/* modern time variable type		*/

	(void)time(&now);

	string title = "Fiber Deform Test";
	// write gnuplot plotting script commands

	//for (j = 0; j < nN; j++) 
	//{ 
	//	// check for three-dimensional frame 
	//	if (xyz[j][0] != 0.0) X = 1;
	//	if (xyz[j][1] != 0.0) Y = 1;
	//	if (xyz[j][2] != 0.0) Z = 1;
	//}
	//
	//if ((X && Y && Z) || D3_flag) 
	//{
	//	D3 = ' '; D2 = '#';
	//}
	//else 
	//{
	//	D3 = '#'; D2 = ' ';
	//}
	// open plotting script file for writing
	if ((fpm = fopen(plotpath, "w")) == NULL)
	{
		sprintf(errMsg, "\n  error: cannot open gnuplot script file: %s \n", plotpath);
		printf(errMsg);
		exit(23);
	}

	// file name for deformed mesh data 
	sprintf(meshfl, "%sf.%03d", meshpath, 0);

	// write header, plot-setup cmds, node label, and element label data

	// header & node number & element number labels

	fprintf(fpm, "# FIBERPRINT FRAME STRUCTUAL ANALYSIS RESULTS  GCL@USTC");
	fprintf(fpm, " VERSION %s \n", std::string(VERSION).c_str());
	fprintf(fpm, "# %s\n", title.c_str());
	fprintf(fpm, "# %s", ctime(&now));
	fprintf(fpm, "# G N U P L O T   S C R I P T   F I L E \n");
	/* fprintf(fpm,"#  X=%d , Y=%d , Z=%d, D3=%d  \n", X,Y,Z,D3_flag); */

	fprintf(fpm, "set autoscale\n");
	fprintf(fpm, "unset border\n");
	fprintf(fpm, "set pointsize 1.0\n");
	fprintf(fpm, "set xtics; set ytics; set ztics; \n");
	fprintf(fpm, "unset zeroaxis\n");
	fprintf(fpm, "unset key\n");
	fprintf(fpm, "unset label\n");
	fprintf(fpm, "set size ratio -1    # 1:1 2D axis scaling \n");
	fprintf(fpm, "# set view equal xyz # 1:1 3D axis scaling \n");

	fprintf(fpm, "# NODE NUMBER LABELS\n");
	for (j = 0; j < nN; j++)
	{
		int o_id = ptr_dualgraph->v_orig_id(j);

		fprintf(fpm, "set label ' %d' at %12.4e, %12.4e, %12.4e\n",
			j + 1, ptr_wf->GetPosition(o_id).x(), ptr_wf->GetPosition(o_id).y(), ptr_wf->GetPosition(o_id).z());
	}

	fprintf(fpm, "# ELEMENT NUMBER LABELS\n");
	for (m = 0; m < nE; m++)
	{
		int e_id = ptr_dualgraph->e_orig_id(m);
		WF_edge *ei = wf_edge_list[e_id];
		int u = ei->ppair_->pvert_->ID();
		int v = ei->pvert_->ID();
		double L = ei->Length();

		int dual_u = ptr_dualgraph->v_dual_id(u);
		int dual_v = ptr_dualgraph->v_dual_id(v);

		mx = 0.5 * (ptr_wf->GetPosition(u).x() + ptr_wf->GetPosition(v).x());
		my = 0.5 * (ptr_wf->GetPosition(u).y() + ptr_wf->GetPosition(v).y());
		mz = 0.5 * (ptr_wf->GetPosition(u).z() + ptr_wf->GetPosition(v).z());
		fprintf(fpm, "set label ' %d' at %12.4e, %12.4e, %12.4e\n",
			m+1, mx, my, mz);
	}

	// 3D plot setup commands

	fprintf(fpm, "%c set parametric\n", D3);
	fprintf(fpm, "%c set view 60, 70, %5.2f \n", D3, scale);
	fprintf(fpm, "%c set view equal xyz # 1:1 3D axis scaling \n", D3);
	fprintf(fpm, "%c unset key\n", D3);
	fprintf(fpm, "%c set xlabel 'x'\n", D3);
	fprintf(fpm, "%c set ylabel 'y'\n", D3);
	fprintf(fpm, "%c set zlabel 'z'\n", D3);
	//	 fprintf(fpm,"%c unset label\n", D3 );

	// different plot title for each load case

	fprintf(fpm, "set title \"%s\\n", title.c_str());
	//fprintf(fpm, "analysis file: %s ", IN_file);

	if (anlyz)
	{
		fprintf(fpm, "  deflection exaggeration: %.1f ", exagg_static);
	}

	fprintf(fpm, "unset clip; \nset clip one; set clip two\n");
	fprintf(fpm, "set xyplane 0 \n"); // requires Gnuplot >= 4.6

	// 2D plot command

	//fprintf(fpm, "%c plot '%s' u 2:3 t 'undeformed mesh' w lp ",
	//	D2, meshpath);
	//if (!anlyz) fprintf(fpm, "lw %d lt 1 pt 6 \n", lw);
	//else fprintf(fpm, "lw 1 lt 5 pt 6, '%s' u 1:2 t 'load case %d of %d' w l lw %d lt 3\n", meshfl, 1, 1, lw);

	// 3D plot command

	fprintf(fpm, "%c splot '%s' u 2:3:4 t 'load case %d of %d' w lp ",
		D3, meshpath, 1, 1);
	if (!anlyz) fprintf(fpm, " lw %d lt 1 pt 6 \n", lw);
	else fprintf(fpm, " lw 1 lt 5 pt 6, '%s' u 1:2:3 t 'load case %d of %d' w l lw %d lt 3\n", meshfl, 1, 1, lw);

	if (anlyz)	fprintf(fpm, "pause -1\n");

	fclose(fpm);

	// write undeformed mesh data

	// open the undeformed mesh data file for writing
	if ((fpm = fopen(meshpath, "w")) == NULL)
	{
		sprintf(errMsg, "\n  error: cannot open gnuplot undeformed mesh data file: %s\n", meshpath);
		printf(errMsg);
		exit(21);
	}

	fprintf(fpm, "# FIBERPRINT FRAME STRUCTURAL ANALYSIS RESULTS  GCL@USTC");
	fprintf(fpm, " VERSION %s \n", std::string(VERSION).c_str());
	fprintf(fpm, "# %s\n", title.c_str());
	fprintf(fpm, "# %s", ctime(&now));
	fprintf(fpm, "# U N D E F O R M E D   M E S H   D A T A   (global coordinates)\n");
	fprintf(fpm, "# Node        X            Y            Z \n");

	for (m = 0; m < nE; m++)
	{
		int e_id = ptr_dualgraph->e_orig_id(m);
		WF_edge *ei = wf_edge_list[e_id];
		int u = ei->ppair_->pvert_->ID();
		int v = ei->pvert_->ID();
		double L = ei->Length();

		int dual_u = ptr_dualgraph->v_dual_id(u);
		int dual_v = ptr_dualgraph->v_dual_id(v);

		mx = 0.5 * (ptr_wf->GetPosition(u).x() + ptr_wf->GetPosition(v).x());
		my = 0.5 * (ptr_wf->GetPosition(u).y() + ptr_wf->GetPosition(v).y());
		mz = 0.5 * (ptr_wf->GetPosition(u).z() + ptr_wf->GetPosition(v).z());

		fprintf(fpm, "%5d %12.4e %12.4e %12.4e \n",
			dual_u+1, ptr_wf->GetPosition(u).x(), ptr_wf->GetPosition(u).y(), ptr_wf->GetPosition(u).z());
		fprintf(fpm, "%5d %12.4e %12.4e %12.4e",
			dual_v+1, ptr_wf->GetPosition(v).x(), ptr_wf->GetPosition(v).y(), ptr_wf->GetPosition(v).z());
		fprintf(fpm, "\n\n\n");
	}
	fclose(fpm);

	if (!anlyz) return; 	// no deformed mesh

	// write deformed mesh data

	// open the deformed mesh data file for writing
	if ((fpm = fopen(meshfl, "w")) == NULL)
	{
		sprintf(errMsg, "\n  error: cannot open gnuplot deformed mesh data file %s \n", meshfl);
		printf(errMsg);
		exit(22);
	}

	fprintf(fpm, "# FIBERPRINT FRAME STRUCTURAL ANALYSIS RESULTS  GCL@USTC");
	fprintf(fpm, " VERSION %s \n", std::string(VERSION).c_str());
	fprintf(fpm, "# %s\n", title.c_str());
	fprintf(fpm, "# %s", ctime(&now));
	fprintf(fpm, "# D E F O R M E D   M E S H   D A T A ");
	fprintf(fpm, "  deflection exaggeration: %.1f\n", exagg_static);
	fprintf(fpm, "#       X-dsp        Y-dsp        Z-dsp\n");


	FILE *fpv = fopen("C:/Users/DELL/Desktop/result/vert.txt", "w");
	FILE *fpl = fopen("C:/Users/DELL/Desktop/result/line.txt", "w");

	for (m = 0; m < nE; m++)
	{
		// write deformed shape data for each element

		ch = 'a';

		int e_id = ptr_dualgraph->e_orig_id(m);
		WF_edge *ei = wf_edge_list[e_id];
		int u = ei->ppair_->pvert_->ID();
		int v = ei->pvert_->ID();
		double L = ei->Length();

		int dual_u = ptr_dualgraph->v_dual_id(u);
		int dual_v = ptr_dualgraph->v_dual_id(v);

		fprintf(fpm, "\n# element %5d \n", m);
		if (anlyz)
		{
			vector<point> beam;
			GnuPltCubicBentBeam(beam,
				D, m, ptr_dualgraph, ptr_wf, exagg_static);

			int nB = beam.size();
			for (int i = 0; i < nB; ++i)
			{
				fprintf(fpm, " %12.4e %12.4e %12.4e\n\n", beam[i].x(), beam[i].y(), beam[i].z());
			}

			if (nB != 0)
			{
				fprintf(fpl, "%lf %lf %lf %lf %lf %lf\n",
					beam[0].x(), beam[0].y(), beam[0].z(),
					beam[nB - 1].x(), beam[nB - 1].y(), beam[nB - 1].z());
				fprintf(fpv, "%lf %lf %lf\n", beam[0].x(), beam[0].y(), beam[0].z());
				fprintf(fpv, "%lf %lf %lf\n", beam[nB - 1].x(), beam[nB - 1].y(), beam[nB - 1].z());
			}
		}
	}

	fclose(fpl);
	fclose(fpv);
	fclose(fpm);
}

/*
* GnuPltCubicBentBeam  -  computes cubic deflection functions from end deflections
* and end rotations.  Saves deflected shapes to a file.  These bent shapes
* are exact for mode-shapes, and for frames loaded at their nodes.
* Nov/25/2015
*/
void StiffnessIO::GnuPltCubicBentBeam(
	vector<point> &beam,
	VX &D,
	int dual_i,
	DualGraph *ptr_dualgraph, WireFrame *ptr_frame,
	double exagg
	)
{
	double	t0, t1, t2, t3, t4, t5, t6, t7, t8, 	/* coord transf matrix entries	*/
		u1, u2, u3, u4, u5, u6, u7, u8, u9, u10, u11, u12,
		s, v, w, dX, dY, dZ;

	WF_edge *ei = ptr_frame->GetEdge(ptr_dualgraph->e_orig_id(dual_i));
	WF_edge *ej = ei->ppair_;
	int dual_u = ptr_dualgraph->v_dual_id(ej->pvert_->ID());
	int dual_v = ptr_dualgraph->v_dual_id(ei->pvert_->ID());

	double L = ei->Length();

	int	i1, i2;
	int info;
	char	errMsg[MAXL];

	MX A(4,4);
	VX a(4);
	VX b(4);

	trsf_.CreateTransMatrix(ej->pvert_->Position(), ei->pvert_->Position(),
		t0, t1, t2, t3, t4, t5, t6, t7, t8, 0.0);

	i1 = 6 * dual_u;	i2 = 6 * dual_v;

	/* compute end deflections in local coordinates */

	u1 = exagg*(t0*D[i1 + 0] + t1*D[i1 + 1] + t2*D[i1 + 2]);
	u2 = exagg*(t3*D[i1 + 0] + t4*D[i1 + 1] + t5*D[i1 + 2]);
	u3 = exagg*(t6*D[i1 + 0] + t7*D[i1 + 1] + t8*D[i1 + 2]);

	u4 = exagg*(t0*D[i1 + 3] + t1*D[i1 + 4] + t2*D[i1 + 5]);
	u5 = exagg*(t3*D[i1 + 3] + t4*D[i1 + 4] + t5*D[i1 + 5]);
	u6 = exagg*(t6*D[i1 + 3] + t7*D[i1 + 4] + t8*D[i1 + 5]);

	u7 = exagg*(t0*D[i2 + 0] + t1*D[i2 + 1] + t2*D[i2 + 2]);
	u8 = exagg*(t3*D[i2 + 0] + t4*D[i2 + 1] + t5*D[i2 + 2]);
	u9 = exagg*(t6*D[i2 + 0] + t7*D[i2 + 1] + t8*D[i2 + 2]);

	u10 = exagg*(t0*D[i2 + 3] + t1*D[i2 + 4] + t2*D[i2 + 5]);
	u11 = exagg*(t3*D[i2 + 3] + t4*D[i2 + 4] + t5*D[i2 + 5]);
	u12 = exagg*(t6*D[i2 + 3] + t7*D[i2 + 4] + t8*D[i2 + 5]);

	/* curve-fitting problem for a cubic polynomial */

	a[0] = u2;		b[0] = u3;
	a[1] = u8;   	b[1] = u9;
	a[2] = u6;		b[2] = -u5;
	a[3] = u12;		b[3] = -u11;

	u7 += L;
	A(0,0) = 1.0;   A(0,1) = u1;   A(0,2) = u1*u1;   A(0,3) = u1*u1*u1;
	A(1,0) = 1.0;   A(1,1) = u7;   A(1,2) = u7*u7;   A(1,3) = u7*u7*u7;
	A(2,0)= 0.0;    A(2,1) = 1.;   A(2,2) = 2.*u1;   A(2,3) = 3.*u1*u1;
	A(3,0) = 0.0;   A(3,1) = 1.;   A(3,2) = 2.*u7;   A(3,3) = 3.*u7*u7;
	u7 -= L;

	VX xa(4);
	info = solver_.LUDecomp(A, xa, a);		/* solve for cubic coef's */

	if (!info)
	{
		sprintf(errMsg, " n1 = %d  n2 = %d  L = %e  u7 = %e \n", dual_u+1, dual_v+1, L, u7);
		printf(errMsg);
		exit(30);
	}

	VX xb(4);
	info = solver_.LUDecomp(A, xb, b);		/* solve for cubic coef's */

	if (!info)
	{
		sprintf(errMsg, " n1 = %d  n2 = %d  L = %e  u7 = %e \n", dual_u + 1, dual_v + 1, L, u7);
		printf(errMsg);
		exit(30);
	}

	// debug ... if deformed mesh exageration is too big, some elements
	// may not be plotted.
	//fprintf( fpm, "# u1=%e  L+u7=%e, dx = %e \n",
	//				u1, fabs(L+u7), fabs(L+u7-u1)/10.0);
	for (s = u1; fabs(s) <= 1.01*fabs(L + u7); s += fabs(L + u7 - u1) / 10.0)
	{

		/* deformed shape in local coordinates */
		v = xa[0] + xa[1] * s + xa[2] * s*s + xa[3] * s*s*s;
		w = xb[0] + xb[1] * s + xb[2] * s*s + xb[3] * s*s*s;

		/* deformed shape in global coordinates */
		dX = t0*s + t3*v + t6*w;
		dY = t1*s + t4*v + t7*w;
		dZ = t2*s + t5*v + t8*w;

		beam.push_back(point(ej->pvert_->Position().x() + dX,
			ej->pvert_->Position().y() + dY, ej->pvert_->Position().z() + dZ));
		//fprintf(fpm, " %12.4e %12.4e %12.4e\n",);
	}

	return;
}


void StiffnessIO::WriteInputData(
	const char *fpath,
	DualGraph *ptr_dualgraph,
	FiberPrintPARM *ptr_parm,
	int verbose
	)
{
	FILE *fp;
	string title_s = "FiberPrint Test File -- static analysis (N,mm,Ton)\n";
	char errMsg[512];

	if ((fp = fopen(fpath, "w")) == NULL)
	{
		sprintf(errMsg, "\n ERROR: cannot open .3dd transfer data file '%s'", fpath);
		printf(errMsg);
		exit(11);
	}

	fprintf(fp, title_s.c_str());
	fprintf(fp, "\n");

	int nN = ptr_dualgraph->SizeOfFaceList();
	int nE = ptr_dualgraph->SizeOfVertList();
	WireFrame *ptr_wf = ptr_dualgraph->ptr_frame_;
	std::vector<WF_edge*> wf_edge_list   = *ptr_wf->GetEdgeList();
	std::vector<DualFace*> dual_face_list = *ptr_dualgraph->GetFaceList();

	double r = ptr_parm->radius_;
	double nr = 0.0;
	double density = ptr_parm->density_;
	double g = ptr_parm->g_;
	double G = ptr_parm->shear_modulus_;
	double E = ptr_parm->youngs_modulus_;
	double v = ptr_parm->poisson_ratio_;

	// Write node data
	fprintf(fp, "%d					# number of nodes\n", nN);
	fprintf(fp, "#.node  x       y       z       r\n");
	fprintf(fp, "#        mm      mm      mm      m\n\n");
	for (int i = 0; i < nN; i++)
	{
		int o_id = ptr_dualgraph->v_orig_id(i);

		fprintf(fp, "%d	 %f  %f  %f  %f\n", i + 1,
			ptr_wf->GetPosition(o_id).x(), ptr_wf->GetPosition(o_id).y(), ptr_wf->GetPosition(o_id).z(),0.0);
	}

	// Write nodes with reactions
	int nR = 0;		// Number of restrained nodes
	std::vector<int>	res_index;
	for (int i = 0; i < nN; i++)
	{
		int id = dual_face_list[i]->orig_id();

		if (ptr_wf->isFixed(id))
		{
			nR++;
			res_index.push_back(i+1);
		}
	}
	fprintf(fp, "\n");
	fprintf(fp, "%d					# number of nodes with reaction\n", nR);
	fprintf(fp, "#.node  x  y  z  xx  yy  zz			1=b_fixed, 0=free\n");
	for (int i = 0; i < nR; i++)
	{
		fprintf(fp, "%d  1  1  1  1  1  1\n", res_index[i]);
	}

	// Write Frame Element data
	double	Ax =  F_PI * r * r;
	double	Asy = Ax * (6 + 12 * v + 6 * v*v) / (7 + 12 * v + 4 * v*v);
	double	Asz = Asy;
	double	Jxx = 0.5 * M_PI * r * r * r * r;
	double  Ksy = 0;		// shear deformation constant
	double  Ksz = 0;

	fprintf(fp, "\n");
	fprintf(fp, "%d					# number of frame element\n", nE);
	fprintf(fp, "#.e n1 n2 Ax    Asy     Asz     Jxx     Iyy     Izz     E		G		roll	density\n");
	fprintf(fp, "#   .  .  mm^2  mm^2    mm^2    mm^4    mm^4    mm^4    MPa		MPa		deg		T/mm^3\n");
	for (int i = 0; i < nE; i++)
	{
		int e_id = ptr_dualgraph->e_orig_id(i);
		WF_edge *ei = wf_edge_list[e_id];
		int u = ei->ppair_->pvert_->ID();
		int v = ei->pvert_->ID();
		double L = ei->Length();

		int dual_u = ptr_dualgraph->v_dual_id(u);
		int dual_v = ptr_dualgraph->v_dual_id(v);

		double Iyy = F_PI * r * r * r * r / 4;
		double Izz = Iyy;

		fprintf(fp, "%d %d %d	%f	%f	%f	%f	%f	%f	%f	%f	%f	%.14f\n",
			i + 1, dual_u + 1, dual_v + 1,
			Ax, Asy, Asz, Jxx, Iyy, Izz, E, G, 0.0, density);
	}

	printf("\n\n");

	// parse option for stiffness matrix
	fprintf(fp, "%d				# 1: include shear deformation\n", 0);
	fprintf(fp, "%d				# 1: include geometric stiffness\n", 0);
	fprintf(fp, "%.1f				# exaggerate static mesh deformation\n", 10.0);
	fprintf(fp, "%.1f				# zoom scale for 3D plotting\n", 2.5);
	fprintf(fp, "%d				# x-axis increment for internal forces, mm\n", -1);
	fprintf(fp, "				# if dx is -1 then internal force calculation are skipped\n");

	// load case parsing
	fprintf(fp, "\n");
	fprintf(fp, "%d				# number of static load cases\n", 1);
	fprintf(fp, "				# Begin static Load Case 1 of 1\n");

	fprintf(fp, "\n");
	fprintf(fp, "# gravitational acceleration for self-weight loading (global)\n");
	fprintf(fp, "#.gX			 gY			     gZ\n");
	fprintf(fp, "#.mm/s^2		 mm/s^2			 mm/s^2\n");
	fprintf(fp, "  0			 0				 %.6f\n", g);

	fprintf(fp,"\n");
	fprintf(fp, "%d				# number of loaded nodes\n",				0);
	fprintf(fp, "%d				# number of uniform loads\n",				0);
	fprintf(fp, "%d				# number of trapezoidal loads\n",			0);
	fprintf(fp, "%d				# number of internal concentrated loads\n", 0);
	fprintf(fp, "%d				# number of temperature loads\n",			0);

	fprintf(fp, "%d				# number of nodes with prescribed displacement\n", nR);
	fprintf(fp, "#.node  X-disp1  Y-disp1  Z-disp1  X-rot'n  Y-rot'n  Z-rot'n\n");
	fprintf(fp, "#       mm		mm	   mm	radian	radian	radian\n");
	for (int i = 0; i < nR; i++)
	{
		fprintf(fp, "%d  0  0  0  0  0  0\n", res_index[i]);
	}

	fprintf(fp, "				# End static Load Case 1 of 1\n");

	// Dynamic Modes
	fprintf(fp, "%d				#number of dynamic modes\n",0);
	fprintf(fp, "# End of transfer data file for fiber test");

	fclose(fp);
}

/*
* SaveUpperMatrix - save a symmetric matrix of dimension [1..n][1..n]	Nov/26/2015
* to the named file, use only upper-triangular part
*/
void StiffnessIO::SaveUpperMatrix(char filename[], const MX &A, int n)
{
	FILE    *fp_m;
	int     i, j;
	time_t	now;

	if ((fp_m = fopen(filename, "w")) == NULL)
	{
		printf(" error: cannot open file: %s \n", filename);
		exit(1016);
	}

	(void)time(&now);
	fprintf(fp_m, "%% filename: %s - %s\n", filename, ctime(&now));
	fprintf(fp_m, "%% type: matrix \n");
	fprintf(fp_m, "%% rows: %d\n", n);
	fprintf(fp_m, "%% columns: %d\n", n);
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			if (i > j)
			{
				if (fabs(A(j,i)) > 1.e-99)
				{
					fprintf(fp_m, "%21.12e", A(j,i));
				}
				else
				{
					fprintf(fp_m, "    0                ");
				}
			}
			else
			{
				if (fabs(A(i, j)) > 1.e-99)
				{
					fprintf(fp_m, "%21.12e", A(i,j));
				}
				else
				{
					fprintf(fp_m, "    0                ");
				}
			}
		}
		fprintf(fp_m, "\n");
	}
	fclose(fp_m);
	return;

}

void StiffnessIO::SaveDisplaceVector(char filename[], const VX &D, int n, DualGraph *ptr_dual_graph)
{
	FILE    *fp;
	int     i, j;
	time_t	now;
	int nN = n / 6;
	vector<DualFace*> dual_face_list = *ptr_dual_graph->GetFaceList();
	VX tmp_D(D.size());

	if ((fp = fopen(filename, "w")) == NULL)
	{
		printf(" error: cannot open file: %s \n", filename);
		exit(1016);
	}

	(void)time(&now);
	fprintf(fp, "%% filename: %s - %s\n", filename, ctime(&now));
	fprintf(fp, "%% type: vector \n");
	fprintf(fp, "%% rows: %d\n", n);

	fprintf(fp, "\n");
	fprintf(fp, "NODE DISPLACEMENTS		(global)\n");
	fprintf(fp, "#.node		X-dsp	Y-dsp	Z-dsp	X-rot	Y-rot	Z-rot\n");
	for (i = 0; i < nN; i++)
	{
		/* i is the dual face id, convert it back to wf_vert id*/
		int v_id = dual_face_list[i]->orig_id();
		int v_dual_id = ptr_dual_graph->v_dual_id(v_id);
		for (j = 0; j < 6; j++)
		{
			tmp_D[6 * v_dual_id + j] = D[6 * i + j];
		}
	}

	for (i = 0; i < nN; i++)
	{
		fprintf(fp, "%d		%.6f		%.6f		%.6f		%.6f		%.6f		%.6f\n",
			i+1, tmp_D[6 * i], tmp_D[6 * i + 1], tmp_D[6 * i + 2], tmp_D[6 * i + 3], tmp_D[6 * i + 4], tmp_D[6 * i + 5]);
	}

	fclose(fp);
	return;

}