#include "choreo_task_sequence_planner/utils/CoordTrans.h"

void CoordTrans::CreateTransMatrix(
	std::vector<V3> xyz,
	double L,			// length of the element(edge)
	int n1, int n2,		// index fo endpoint of the element
	double &t0, double &t1, double &t2, double &t3, double &t4,
	double &t5, double &t6, double &t7, double &t8,
	float p)
{
	//  CoordTrans - calculate the 9 elements of the block - diagonal 12 - by - 12
	//	coordinate transformation matrix, t1, t2, ..., t9.

	//	These coordinate transformation factors are used to :
	//  transform frame element end forces from the element(local) coordinate system
	//	to the structral(global) coordinate system.

	//	Element matrix coordinate transformations are carried out by function ATMA

	double	Cx, Cy, Cz, den,		/* direction cosines	*/
		Cp, Sp;			/* cosine and sine of roll angle */

	Cx = (xyz[n2][0] - xyz[n1][0]) / L;
	Cy = (xyz[n2][1] - xyz[n1][1]) / L;
	Cz = (xyz[n2][2] - xyz[n1][2]) / L;

	t0 = t1 = t2 = t3 = t4 = t5 = t6 = t7 = t8 = 0.0;

	Cp = cos(p);
	Sp = sin(p);

	if (fabs(Cz) == 1.0)
	{
		t2 = Cz;
		t3 = -Cz*Sp;
		t4 = Cp;
		t6 = -Cz*Cp;
		t7 = -Sp;
	}
	else
	{

		den = sqrt(1.0 - Cz*Cz);

		t0 = Cx;
		t1 = Cy;
		t2 = Cz;

		t3 = (-Cx*Cz*Sp - Cy*Cp) / den;
		t4 = (-Cy*Cz*Sp + Cx*Cp) / den;
		t5 = Sp*den;

		t6 = (-Cx*Cz*Cp + Cy*Sp) / den;
		t7 = (-Cy*Cz*Cp - Cx*Sp) / den;
		t8 = Cp*den;
	}
}


void CoordTrans::CreateTransMatrix(
	point u, point v,
	double &t0, double &t1, double &t2, double &t3, double &t4,
	double &t5, double &t6, double &t7, double &t8,
	float p)
{
	double L = sqrt((u.x() - v.x())*(u.x() - v.x()) + (u.y() - v.y())*(u.y() - v.y())
						+ (u.z() - v.z())*(u.z() - v.z()));

	double	Cx, Cy, Cz, den,		/* direction cosines	*/
			Cp, Sp;					/* cosine and sine of roll angle */

	Cx = (v.x() - u.x()) / L;
	Cy = (v.y() - u.y()) / L;
	Cz = (v.z() - u.z()) / L;

	t0 = t1 = t2 = t3 = t4 = t5 = t6 = t7 = t8 = 0.0;

	Cp = cos(p);
	Sp = sin(p);

	if (fabs(Cz) == 1.0)
	{
		t2 = Cz;
		t3 = -Cz*Sp;
		t4 = Cp;
		t6 = -Cz*Cp;
		t7 = -Sp;
	}
	else
	{
		den = sqrt(1.0 - Cz*Cz);

		t0 = Cx;
		t1 = Cy;
		t2 = Cz;

		t3 = (-Cx*Cz*Sp - Cy*Cp) / den;
		t4 = (-Cy*Cz*Sp + Cx*Cp) / den;
		t5 = Sp*den;

		t6 = (-Cx*Cz*Cp + Cy*Sp) / den;
		t7 = (-Cy*Cz*Cp - Cx*Sp) / den;
		t8 = Cp*den;
	}
}


void CoordTrans::TransLocToGlob(
	double t0, double t1, double t2, double t3, double t4,
	double t5, double t6, double t7, double t8,
	MX	   &m, float r1, float r2)
{
	int     i, j, k;

	MX t(12, 12);
	MX mt(12, 12);

	t.setZero();
	mt.setZero();

	for (i = 0; i < 4; i++)
	{
		t(3 * i, 3 * i) = t0;
		t(3 * i, 3 * i + 1) = t1;
		t(3 * i, 3 * i + 2) = t2;
		t(3 * i + 1, 3 * i) = t3;
		t(3 * i + 1, 3 * i + 1) = t4;
		t(3 * i + 1, 3 * i + 2) = t5;
		t(3 * i + 2, 3 * i) = t6;
		t(3 * i + 2, 3 * i + 1) = t7;
		t(3 * i + 2, 3 * i + 2) = t8;
	}

	/*  effect of finite node radius on coordinate transformation  is not supported now */

	mt = t.transpose() * m * t;
	m = mt;
}