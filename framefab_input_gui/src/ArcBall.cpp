// ArcBall.cpp: implementation of the CArcBall class.
//
//////////////////////////////////////////////////////////////////////
#include "framefab_input_gui/ArcBall.h"

#include <math.h>

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CArcBall::CArcBall()
{
	int	i, j;

	for( i = 0; i < 4; i ++ )
	{
		for( j = 0; j < 4; j ++ )
		{
			if( i == j )
				mId[i][j] = 1.f;
			else	mId[i][j] = 0.f;
		}
	}

	otherAxis[0][0] = -0.48f;
	otherAxis[0][1] = 0.80f;
	otherAxis[0][2] = 0.36f;
	otherAxis[0][3] = 1.0f;

	qOne.x = qOne.y = qOne.z = 0.f;
	qOne.w = 1.f;
/*####################################*/
        m_nWinWidth = 640;
        m_nWinHeight = 480;

        InitBall();
/*#####################################*/
}

CArcBall::CArcBall(int winWidth,int winHeight)
    :m_nWinWidth(winWidth),m_nWinHeight(winHeight)
{
    int	i, j;

    for( i = 0; i < 4; i ++ )
    {
            for( j = 0; j < 4; j ++ )
            {
                    if( i == j )
                            mId[i][j] = 1.f;
                    else	mId[i][j] = 0.f;
            }
    }

    otherAxis[0][0] = -0.48f;
    otherAxis[0][1] = 0.80f;
    otherAxis[0][2] = 0.36f;
    otherAxis[0][3] = 1.0f;

    qOne.x = qOne.y = qOne.z = 0.f;
    qOne.w = 1.f;

    InitBall();
}


CArcBall::~CArcBall()
{

}

/////////////////////////////////////////////////////////////////////
// aux functions

/* Return quaternion product qL * qR.  Note: order is important!
* To combine rotations, use the product Mul(qSecond, qFirst),
* which gives the effect of rotating by qFirst then qSecond. */
CArcBall::Quat_t CArcBall::Qt_Mul(Quat_t qL, Quat_t qR)
{
	Quat_t qq;
	qq.w = qL.w*qR.w - qL.x*qR.x - qL.y*qR.y - qL.z*qR.z;
	qq.x = qL.w*qR.x + qL.x*qR.w + qL.y*qR.z - qL.z*qR.y;
	qq.y = qL.w*qR.y + qL.y*qR.w + qL.z*qR.x - qL.x*qR.z;
	qq.z = qL.w*qR.z + qL.z*qR.w + qL.x*qR.y - qL.y*qR.x;
	return (qq);
}


/* Construct rotation matrix from (possibly non-unit) quaternion.
* Assumes matrix is used to multiply column vector on the left:
* vnew = mat vold.  Works correctly for right-handed coordinate system
* and right-handed rotations. */
CArcBall::HMatrix * CArcBall::Qt_ToMatrix(Quat_t q, HMatrix& out)
{
	double Nq = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
	double s = (Nq > 0.0) ? (2.0 / Nq) : 0.0;
	double xs = q.x*s,	      ys = q.y*s,	  zs = q.z*s;
	double wx = q.w*xs,	      wy = q.w*ys,	  wz = q.w*zs;
	double xx = q.x*xs,	      xy = q.x*ys,	  xz = q.x*zs;
	double yy = q.y*ys,	      yz = q.y*zs,	  zz = q.z*zs;
	out[X][X] = float(1.0 - (yy + zz)); out[Y][X] = float(xy + wz); out[Z][X] = float(xz - wy);
	out[X][Y] = float(xy - wz); out[Y][Y] = float(1.0 - (xx + zz)); out[Z][Y] = float(yz + wx);
	out[X][Z] = float(xz + wy); out[Y][Z] = float(yz - wx); out[Z][Z] = float(1.0 - (xx + yy));
	out[X][W] = out[Y][W] = out[Z][W] = out[W][X] = out[W][Y] = out[W][Z] = 0.0f;
	out[W][W] = 1.0f;
	return ((HMatrix *)&out);
}

/* Return conjugate of quaternion. */
CArcBall::Quat_t CArcBall::Qt_Conj(Quat_t q)
{
	Quat_t qq;
	qq.x = -q.x; qq.y = -q.y; qq.z = -q.z; qq.w = q.w;
	return (qq);
}

/* Return vector formed from components */
CArcBall::HVect CArcBall::V3_(float x, float y, float z)
{
	HVect v;
	v.x = x; v.y = y; v.z = z; v.w = 0.0f;
	return (v);
}

/* Return norm of v, defined as sum of squares of components */
float CArcBall::V3_Norm(HVect v)
{
	return ( v.x*v.x + v.y*v.y + v.z*v.z );
}

/* Return unit magnitude vector in direction of v */
CArcBall::HVect CArcBall::V3_Unit(HVect v)
{
	static HVect u = {0.0f, 0.0f, 0.0f, 0.0f};
	float vlen = float(sqrt(V3_Norm(v)));
	if (vlen != 0.0f) {
		u.x = v.x/vlen; u.y = v.y/vlen; u.z = v.z/vlen;
	}
	return (u);
}

/* Return version of v scaled by s */
CArcBall::HVect CArcBall::V3_Scale(HVect v, float s)
{
	HVect u;
	u.x = s*v.x; u.y = s*v.y; u.z = s*v.z; u.w = v.w;
	return (u);
}

/* Return negative of v */
CArcBall::HVect CArcBall::V3_Negate(HVect v)
{
	static HVect u = {0.0f, 0.0f, 0.0f, 0.0f};
	u.x = -v.x; u.y = -v.y; u.z = -v.z;
	return (u);
}

/* Return sum of v1 and v2 */
CArcBall::HVect CArcBall::V3_Add(HVect v1, HVect v2)
{
	static HVect v = {0.0f, 0.0f, 0.0f, 0.0f};
	v.x = v1.x+v2.x; v.y = v1.y+v2.y; v.z = v1.z+v2.z;
	return (v);
}

/* Return difference of v1 minus v2 */
CArcBall::HVect CArcBall::V3_Sub(HVect v1, HVect v2)
{
	static HVect v = {0.0f, 0.0f, 0.0f, 0.0f};
	v.x = v1.x-v2.x; v.y = v1.y-v2.y; v.z = v1.z-v2.z;
	return (v);
}

/* Halve arc between unit vectors v0 and v1. */
CArcBall::HVect CArcBall::V3_Bisect(HVect v0, HVect v1)
{
	static HVect v = {0.0f, 0.0f, 0.0f, 0.0f};
	float Nv;
	v = V3_Add(v0, v1);
	Nv = V3_Norm(v);
	if (Nv < 1.0e-5) {
		v = V3_(0.0f, 0.0f, 1.0f);
	} else {
		v = V3_Scale(v, float(1/sqrt(Nv)));
	}
	return (v);
}

/* Return dot product of v1 and v2 */
float CArcBall::V3_Dot(HVect v1, HVect v2)
{
	return (v1.x*v2.x + v1.y*v2.y + v1.z*v2.z);
}


/* Return cross product, v1 x v2 */
CArcBall::HVect CArcBall::V3_Cross(HVect v1, HVect v2)
{
	static HVect v = {0.0f, 0.0f, 0.0f, 0.0f};
	v.x = v1.y*v2.z-v1.z*v2.y;
	v.y = v1.z*v2.x-v1.x*v2.z;
	v.z = v1.x*v2.y-v1.y*v2.x;
	return (v);
}


/////////////////////////////////////////////////////////////////////
// math functions : Essential routines for ArcBall.  ****/
/* Convert window coordinates to sphere coordinates. */
CArcBall::HVect CArcBall::MouseOnSphere(HVect mouse, HVect ballCenter, double ballRadius)
{
	HVect ballMouse;
	register double mag;
	ballMouse.x = float((mouse.x - ballCenter.x) / ballRadius);
	ballMouse.y = float((mouse.y - ballCenter.y) / ballRadius);
	mag = ballMouse.x*ballMouse.x + ballMouse.y*ballMouse.y;
	if (mag > 1.0) {
		register float scale = float(1.0/sqrt(mag));
		ballMouse.x *= scale; ballMouse.y *= scale;
		ballMouse.z = 0.0f;
	} else {
		ballMouse.z = float(sqrt(1 - mag));
	}
	ballMouse.w = 0.0f;
	return (ballMouse);
}

/* Construct a unit quaternion from two points on unit sphere */
CArcBall::Quat_t CArcBall::Qt_FromBallPoints(HVect from, HVect to)
{
	Quat_t qu;
	qu.x = from.y*to.z - from.z*to.y;
	qu.y = from.z*to.x - from.x*to.z;
	qu.z = from.x*to.y - from.y*to.x;
	qu.w = from.x*to.x + from.y*to.y + from.z*to.z;
	return (qu);
}

/* Convert a unit quaternion to two points on unit sphere */
void CArcBall::Qt_ToBallPoints(Quat_t q, HVect *arcFrom, HVect *arcTo)
{
	double s;
	s = sqrt(q.x*q.x + q.y*q.y);
	if (s == 0.0) {
		*arcFrom = V3_(0.0f, 1.0f, 0.0f);
	} else {
		*arcFrom = V3_(float(-q.y/s), float(q.x/s), 0.0f);
	}
	arcTo->x = q.w*arcFrom->x - q.z*arcFrom->y;
	arcTo->y = q.w*arcFrom->y + q.z*arcFrom->x;
	arcTo->z = q.x*arcFrom->y - q.y*arcFrom->x;
	if (q.w < 0.0f) *arcFrom = V3_(-arcFrom->x, -arcFrom->y, 0.0f);
}

/* Force sphere point to be perpendicular to axis. */
CArcBall::HVect CArcBall::ConstrainToAxis(HVect loose, HVect axis)
{
	HVect onPlane;
	register float norm;
	onPlane = V3_Sub(loose, V3_Scale(axis, V3_Dot(axis, loose)));
	norm = V3_Norm(onPlane);
	if (norm > 0.0) {
		if (onPlane.z < 0.0) onPlane = V3_Negate(onPlane);
		return ( V3_Scale(onPlane, float(1/sqrt(norm))) );
	} /* else drop through */
	if (axis.z == 1) {
		onPlane = V3_(1.0f, 0.0f, 0.0f);
	} else {
		onPlane = V3_Unit(V3_(-axis.y, axis.x, 0.0f));
	}
	return (onPlane);
}

/* Find the index of nearest arc of axis set. */
int CArcBall::NearestConstraintAxis(HVect loose, HVect *axes, int nAxes)
{
	HVect onPlane;
	register float max, dot;
	register int i, nearest;
	max = -1.0f; nearest = 0;
	for (i=0; i<nAxes; i++) {
		onPlane = ConstrainToAxis(loose, axes[i]);
		dot = V3_Dot(onPlane, loose);
		if (dot>max) {
			max = dot; nearest = i;
		}
	}
	return (nearest);
}


/////////////////////////////////////////////////////////////////////
// ball functions

/* Establish reasonable initial values for controller. */
void CArcBall::Init()
{
	int i;
	ball_data.center = qOne;
	ball_data.radius = 1.0f;
	ball_data.vDown = ball_data.vNow = qOne;
	ball_data.qDown = ball_data.qNow = qOne;
	for (i=15; i>=0; i--)
		((float *)ball_data.mNow)[i] = ((float *)ball_data.mDown)[i] = ((float *)ball_data.mDeltaNow)[i] = ((float *)mId)[i];
	ball_data.showResult = ball_data.dragging = false;
	ball_data.axisSet = NoAxes;
	ball_data.sets[CameraAxes] = mId[X]; ball_data.setSizes[CameraAxes] = 3;
	ball_data.sets[BodyAxes] = ball_data.mDown[X]; ball_data.setSizes[BodyAxes] = 3;
	ball_data.sets[OtherAxes] = otherAxis[X]; ball_data.setSizes[OtherAxes] = 1;
}

/* Set the center and size of the controller. */
void CArcBall::Place(BallData *ball, HVect center, double radius)
{
	ball->center = center;
	ball->radius = radius;
}

/* Incorporate new mouse position. */
void CArcBall::Mouse(HVect vNow)
{
	ball_data.vNow = vNow;
}

/* Choose a constraint set, or none. */
void CArcBall::UseSet(BallData *ball, AxisSet axisSet)
{
	if (!ball->dragging) ball->axisSet = axisSet;
}

/* Begin drawing arc for all drags combined. */
void CArcBall::ShowResult(BallData *ball)
{
	ball->showResult = true;
}

/* Stop drawing arc for all drags combined. */
void CArcBall::HideResult(BallData *ball)
{
	ball->showResult = false;
}

/* Using vDown, vNow, dragging, and axisSet, compute rotation etc. */
void CArcBall::Update()
{
	int setSize = ball_data.setSizes[ball_data.axisSet];
	HVect *set = (HVect *)(ball_data.sets[ball_data.axisSet]);
	ball_data.vFrom = MouseOnSphere(ball_data.vDown, ball_data.center, ball_data.radius);
	ball_data.vTo = MouseOnSphere(ball_data.vNow, ball_data.center, ball_data.radius);
	//  ball->vDown = ball->vNow; // update vDown
	if (ball_data.dragging)
	{
		if (ball_data.axisSet!=NoAxes)
		{
			ball_data.vFrom = ConstrainToAxis(ball_data.vFrom, set[ball_data.axisIndex]);
			ball_data.vTo = ConstrainToAxis(ball_data.vTo, set[ball_data.axisIndex]);
		}
		ball_data.qDrag = Qt_FromBallPoints(ball_data.vFrom, ball_data.vTo);
		ball_data.qNow = Qt_Mul(ball_data.qDrag, ball_data.qDown);
		ball_data.qDrag = ball_data.qNow;
		//ball->qNow = ball->qDrag;
	}
	else
	{
		if (ball_data.axisSet!=NoAxes)
		{
			ball_data.axisIndex = NearestConstraintAxis(ball_data.vTo, set, setSize);
		}
	}
	Qt_ToBallPoints(ball_data.qDown, &ball_data.vrFrom, &ball_data.vrTo);
	
	/* Gives transpose for GL. */
	Qt_ToMatrix(Qt_Conj(ball_data.qNow), ball_data.mNow); 
	Qt_ToMatrix(Qt_Conj(ball_data.qDrag), ball_data.mDeltaNow); 
}

/* Return rotation matrix defined by controller use. */
void CArcBall::Value()
{
	int i;
	for (i=15; i>=0; i--)
		( (float *)ball_matrix)[i] = ((float *)ball_data.mDeltaNow)[i];
}


/* Begin drag sequence. */
void CArcBall::BeginDrag()
{
	//ball->qDown = qOne;
	ball_data.dragging = true;
	ball_data.vDown = ball_data.vNow;
}

/* Stop drag sequence. */
void CArcBall::EndDrag()
{
	int i;
	ball_data.dragging = false;
	ball_data.qDown = ball_data.qNow;
	for (i=15; i>=0; i--)
		((float *)ball_data.mDown)[i] = ((float *)ball_data.mNow)[i];
}

CArcBall::HVect CArcBall::convert(int x, int y, int nWinWidth, int nWinHeight)
{
	HVect coord;
	if(nWinWidth>=nWinHeight)
	{
		coord.x=(float(x)/nWinWidth-0.5f)*2*(nWinWidth/nWinHeight);
		coord.y=-(float(y)/nWinHeight-0.5f)*2;
	}
	else
	{
		coord.x=(float(x)/nWinWidth-0.5f)*2;
		coord.y=-(float(y)/nWinHeight-0.5f)*2*(nWinHeight/nWinWidth);
	}
	return coord;
}
/*
void CArcBall::MouseDown(POINT point, int nWinWidth, int nWinHeight)
{
    MouseDown(point.x, point.y,nWinWidth, nWinHeight);
}


void CArcBall::MouseUp(POINT point, int nWinWidth, int nWinHeight)
{
    MouseUp(point.x, point.y,nWinWidth, nWinHeight);
}

void CArcBall::MouseMove(POINT point, int nWinWidth, int nWinHeight)
{
    MouseMove(point.x, point.y,nWinWidth, nWinHeight);
}
*/
void CArcBall::InitBall()
{
	Init();
	Value();
}

float * CArcBall::GetInvertedBallMatrix()
{
	static float inverse[16];
	int i, j, k, swap;
	double t;
	float temp[4][4];
	float *src;

	src = GetBallMatrix();
	
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			temp[i][j] = src[i * 4 + j];
		}
	}

	MakeIdentity(inverse);
	
	for (i = 0; i < 4; i++)
	{
		/* 
		 ** Look for largest element in column */
		swap = i;
		for (j = i + 1; j < 4; j++)
		{
			if (fabs(temp[j][i]) > fabs(temp[i][i]))
			{
				swap = j;
			}
		}
		
		if (swap != i)
		{
			/* 
			** Swap rows. */
			for (k = 0; k < 4; k++)
			{
				t = temp[i][k];
				temp[i][k] = temp[swap][k];
				temp[swap][k] = (float)t;
				
				t = inverse[i * 4 + k];
				inverse[i * 4 + k] = inverse[swap * 4 + k];
				inverse[swap * 4 + k] = (float)t;
			}
		}
		if (temp[i][i] == 0)
		{
		/* 
		** No non-zero pivot.  The matrix is singular, which
		shouldn't ** happen.  This means the user gave us a
			bad matrix. */
			return inverse;
		}
		t = temp[i][i];
		for (k = 0; k < 4; k++)
		{
			temp[i][k] /= (float)t;
			inverse[i * 4 + k] /= (float)t;
		}
		for (j = 0; j < 4; j++)
		{
			if (j != i)
			{
				t = temp[j][i];
				for (k = 0; k < 4; k++)
				{
					temp[j][k] -= temp[i][k] * (float)t;
					inverse[j * 4 + k] -= inverse[i * 4 + k] * (float)t;
				}
			}
		}
	}
	return (inverse);
}


void CArcBall::MakeIdentity(float fpTheMatrix[])
{
	fpTheMatrix[0 + 4 * 0] = 1;
	fpTheMatrix[0 + 4 * 1] = 0;
	fpTheMatrix[0 + 4 * 2] = 0;
	fpTheMatrix[0 + 4 * 3] = 0;
	fpTheMatrix[1 + 4 * 0] = 0;
	fpTheMatrix[1 + 4 * 1] = 1;
	fpTheMatrix[1 + 4 * 2] = 0;
	fpTheMatrix[1 + 4 * 3] = 0;
	fpTheMatrix[2 + 4 * 0] = 0;
	fpTheMatrix[2 + 4 * 1] = 0;
	fpTheMatrix[2 + 4 * 2] = 1;
	fpTheMatrix[2 + 4 * 3] = 0;
	fpTheMatrix[3 + 4 * 0] = 0;
	fpTheMatrix[3 + 4 * 1] = 0;
	fpTheMatrix[3 + 4 * 2] = 0;
	fpTheMatrix[3 + 4 * 3] = 1;
}

void CArcBall::reSetBound(int winwidth, int winheight)
{
    m_nWinWidth = winwidth;
    m_nWinHeight = winheight;
}

void CArcBall::MouseMove(int px, int py, int nWinWidth, int nWinHeight)
{
        HVect v = convert(px,py, nWinWidth, nWinHeight);

        Mouse(v);	// set the mouse position
        Update();	// Alters the internal state of the arcball
        Value();	// reads the matrix from the arcball
}

void CArcBall::MouseDown(int px, int py, int nWinWidth, int nWinHeight)
{
    HVect v = convert(px, py, nWinWidth, nWinHeight);

    Mouse(v);	// sets new mouse coordinates
    BeginDrag();	// indictes that dragging should begin
    Update();
}

void CArcBall::MouseUp(int px, int py, int nWinWidth, int nWinHeight)
{
    HVect v = convert(px, py, nWinWidth, nWinHeight);

    Mouse( v);	// sets new mouse coordinates
    EndDrag();	// indictes that dragging should end
    Update();	// Alters the internal state of the arcball
    Value();
}
