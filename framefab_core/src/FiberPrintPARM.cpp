#include "FiberPrintPARM.h"


FiberPrintPARM::FiberPrintPARM(
	double Wp, double Wa, double Wi, double seq_D_tol,
	double ADMM_D_tol , double penalty, double pri_tol, double dual_tol,
	double radius, double density, double g, 
	double youngs_modulus, double shear_modulus, double poisson_ratio
	)
{
	/*
	*	For Your Inference; Unit Transfer
	*	Gpa = 10^3Mpa = 10^9pa
	*   1 kg/m^3 = 1e-3 g/cm^3 = 1e-12 Ton/mm^3
	*	1 Pa = 1 N/mm^2
	*	1 N  = kg/(m * s^2)
	*	1 Giga = 10^3 Mega = 10^6
	*/
	
	/* 
	* Test case 1: Acrylonitrile Butadiene Styrene (ABS)
	* Data Source : http://www.grantadesign.com/education/datasheets/ABS.htm
	* Density ;						1210 * 1e-12 Ton/mm^3
	* Elastic(Young's) Modulus :    1100 Mpa
	* Shear Modulus :				1032 Mpa
	* radius of the element :		0.6	 mm
	* poisson ratio :				0.39
	*/

	/*
	* Gravity acceleration along Z axis:   gZ = -9806.33 mm/s^2
	*/

	radius_ = radius;
	density_ = density;
	g_ = g;
	youngs_modulus_ = youngs_modulus;
	shear_modulus_ = shear_modulus;
	poisson_ratio_ = poisson_ratio;

	ADMM_D_tol_ = ADMM_D_tol;
	penalty_ = penalty;
	pri_tol_ = pri_tol;
	dual_tol_ = dual_tol;

	Wp_ = Wp;
	Wa_ = Wa;
	Wi_ = Wi;
	seq_D_tol_ = seq_D_tol;
}


FiberPrintPARM::~FiberPrintPARM()
{
}
