/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	CArcBall
*
*		Description:	Visualization widget class
*
*		Created:  Oct/10/2015
*
*		Author:  Xin Hu, Yijiang Huang, Guoxian Song
*		Company:  GCL@USTC
----------------------------------------------------------------------------
*		Copyright (C) 2016  Yijiang Huang, Xin Hu, Guoxian Song, Juyong Zhang
*		and Ligang Liu.
*
*		FrameFab is free software: you can redistribute it and/or modify
*		it under the terms of the GNU General Public License as published by
*		the Free Software Foundation, either version 3 of the License, or
*		(at your option) any later version.
*
*		FrameFab is distributed in the hope that it will be useful,
*		but WITHOUT ANY WARRANTY; without even the implied warranty of
*		MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*		GNU General Public License for more details.
*
*		You should have received a copy of the GNU General Public License
*		along with FrameFab.  If not, see <http://www.gnu.org/licenses/>.
* ==========================================================================
*/

#include <math.h>
#include <QtOpenGL/QGLWidget>
#include <QPoint>

#ifndef _MY_ARCBALL_CLASS_
#define _MY_ARCBALL_CLASS_


class CArcBall  
{
public:
	typedef enum AxisSet
	{
		NoAxes, CameraAxes, BodyAxes, OtherAxes, NSets
        }AxisSet;
private:
	typedef struct Quat_t
	{
		float x, y, z, w;
	}Quat_t;

	typedef Quat_t HVect;

	enum QuatPart
	{
		X, Y, Z, W, QuatLen
	};

	typedef float HMatrix[QuatLen][QuatLen];

	typedef float *ConstraintSet;

	typedef struct BallData
	{
		HVect		center;
		double		radius;
		Quat_t		qNow, qDown, qDrag;
		HVect		vNow, vDown, vFrom, vTo, vrFrom, vrTo;
		HMatrix		mNow, mDown, mDeltaNow;
		bool		showResult, dragging;
		ConstraintSet	sets[NSets];
		int		setSizes[NSets];
		AxisSet		axisSet;
		int		axisIndex;
        }BallData;

private: // variables
	HMatrix mId;
	float otherAxis[1][4];
	Quat_t qOne;

	BallData ball_data;
	HMatrix ball_matrix;

public:

private: // functions
	void MakeIdentity(float *fpTheMatrix);
	// aux
	HMatrix * Qt_ToMatrix(Quat_t q, HMatrix& out);
	Quat_t Qt_Conj(Quat_t q);
	Quat_t Qt_Mul(Quat_t qL, Quat_t qR);
	HVect V3_(float x, float y, float z);
	float V3_Norm(HVect v);
	HVect V3_Unit(HVect v);
	HVect V3_Scale(HVect v, float s);
	HVect V3_Negate(HVect v);
	HVect V3_Add(HVect v1, HVect v2);
	HVect V3_Sub(HVect v1, HVect v2);
	float V3_Dot(HVect v1, HVect v2);
	HVect V3_Cross(HVect v1, HVect v2);
	HVect V3_Bisect(HVect v0, HVect v1);
	
	// math
	HVect MouseOnSphere(HVect mouse, HVect ballCenter, double ballRadius);
	HVect ConstrainToAxis(HVect loose, HVect axis);
	int NearestConstraintAxis(HVect loose, HVect *axes, int nAxes);
	Quat_t Qt_FromBallPoints(HVect from, HVect to);
	void Qt_ToBallPoints(Quat_t q, HVect *arcFrom, HVect *arcTo);
	
	// ball
	void Place(BallData *ball, HVect center, double radius);
	void UseSet(BallData *ball, AxisSet axisSet);
	void ShowResult(BallData *ball);
	void HideResult(BallData *ball);
	void Value();
	void Mouse(HVect vNow);
	void Update();
	void BeginDrag();
	void EndDrag();
	HVect convert(int x, int y, int nWinWidth, int nWinHeight);
	void Init();
	
public:
	float * GetInvertedBallMatrix();
	void InitBall();

private:
     void MouseMove(int px, int py, int nWinWidth, int nWinHeight);
      void MouseDown(int px, int py, int nWinWidth, int nWinHeight);
      void MouseUp(int px, int py, int nWinWidth, int nWinHeight);

public:
 //       void MouseMove(POINT point, int nWinWidth, int nWinHeight);
 //      void MouseMove(POINT point);
 //       void MouseUp(POINT point, int nWinWidth, int nWinHeight);
 //       void MouseUp(POINT point);
 //       void MouseDown( POINT point, int nWinWidth, int nWinHeight);
 //      void MouseDown(POINT point);
	float * GetBallMatrix()
	{
		float * ptr = (float*)ball_matrix;
		return	ptr;
	}
	CArcBall();
	virtual ~CArcBall();
/*############################################################*/
private:
        int m_nWinWidth;
        int m_nWinHeight;

public:
        void reSetBound(int winwidth, int winheight);
        CArcBall(int winWidth, int winHeight);

        int getWinWidth()const {return m_nWinWidth;}
        int getWinHeight()const{return m_nWinHeight;}

        void MouseDown(QPoint point)
        {
            MouseDown(point.x(),point.y(),m_nWinWidth, m_nWinHeight);
        }
        void MouseUp(QPoint point)
        {
            MouseUp(point.x(),point.y(),m_nWinWidth, m_nWinHeight);
        }
        void MouseMove(QPoint point)
        {
            MouseMove(point.x(),point.y(),m_nWinWidth, m_nWinHeight);
        }

/*################################################################*/
};

#endif


