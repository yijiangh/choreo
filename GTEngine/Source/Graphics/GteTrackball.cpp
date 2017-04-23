// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteTrackball.h>
#include <LowLevel/GteLogger.h>
using namespace gte;


Trackball::Trackball()
    :
    mXSize(0),
    mYSize(0),
    mMultiplier(0.0f),
    mX0(0.0f),
    mY0(0.0f),
    mX1(0.0f),
    mY1(0.0f),
    mActive(false),
    mValidTrackball(false)
{
    mRoot = std::make_shared<Node>();
    mInitialOrientation.MakeIdentity();
}

Trackball::Trackball(int xSize, int ySize,
    std::shared_ptr<Camera> const& camera)
    :
    mXSize(0),
    mYSize(0),
    mMultiplier(0.0f),
    mX0(0.0f),
    mY0(0.0f),
    mX1(0.0f),
    mY1(0.0f),
    mActive(false),
    mValidTrackball(false)
{
    Set(xSize, ySize, camera);
    mRoot = std::make_shared<Node>();
    mInitialOrientation.MakeIdentity();
}

void Trackball::Set(int xSize, int ySize,
    std::shared_ptr<Camera> const& camera)
{
    if (xSize > 0 && ySize > 0 && camera)
    {
        mXSize = xSize;
        mYSize = ySize;
        mCamera = camera;
        mMultiplier = 1.0f / (mXSize >= mYSize ? mYSize : mXSize);
        mX0 = 0.5f * mXSize;
        mY0 = 0.5f * mYSize;
        mX1 = mX0;
        mY1 = mY0;
        mValidTrackball = true;
    }
    else
    {
        LogError("Invalid Trackball parameters.");
        mValidTrackball = false;
    }
}

void Trackball::Attach(std::shared_ptr<Spatial> const& object)
{
    if (mValidTrackball && object)
    {
        mRoot->AttachChild(object);
    }
}

void Trackball::Detach(std::shared_ptr<Spatial> const& object)
{
    if (mValidTrackball && object)
    {
        mRoot->DetachChild(object);
    }
}

void Trackball::DetachAll()
{
    mRoot->DetachAllChildren();
}

void Trackball::SetInitialPoint(int x, int y)
{
    if (mValidTrackball)
    {
        mX0 = (2.0f * x - mXSize) * mMultiplier;
        mY0 = (2.0f * y - mYSize) * mMultiplier;
        mInitialOrientation = mRoot->worldTransform.GetRotation();
    }
}

void Trackball::SetFinalPoint(int x, int y)
{
    if (mValidTrackball)
    {
        mX1 = (2.0f * x - mXSize) * mMultiplier;
        mY1 = (2.0f * y - mYSize) * mMultiplier;
        if (mX1 != mX0 || mY1 != mY0)
        {
            UpdateOrientation();
        }
    }
}

void Trackball::UpdateOrientation()
{
    // Get the first vector on the sphere.
    float sqrLength0 = mX0 * mX0 + mY0 * mY0;
    float length0 = sqrt(sqrLength0), invLength0 = 0.0f, z0, z1;
    if (length0 > 1.0f)
    {
        // Outside the unit disk, project onto it.
        invLength0 = 1.0f / length0;
        mX0 *= invLength0;
        mY0 *= invLength0;
        z0 = 0.0f;
    }
    else
    {
        // Compute point (mX0,mY0,z0) on negative unit hemisphere.
        z0 = 1.0f - sqrLength0;
        z0 = (z0 <= 0.0f ? 0.0f : sqrt(z0));
    }
    z0 *= -1.0f;

    // Use camera world coordinates, order is (D,U,R), so point is (z,y,x).
    Vector4<float> vec0{ z0, mY0, mX0, 0.0f };

    // Get the second vector on the sphere.
    float sqrLength1 = mX1 * mX1 + mY1 * mY1;
    float length1 = sqrt(sqrLength1), invLength1 = 0.0f;
    if (length1 > 1.0f)
    {
        // Outside unit disk, project onto it.
        invLength1 = 1.0f / length1;
        mX1 *= invLength1;
        mY1 *= invLength1;
        z1 = 0.0f;
    }
    else
    {
        // Compute point (mX1,mY1,z1) on negative unit hemisphere.
        z1 = 1.0f - sqrLength1;
        z1 = (z1 <= 0.0f ? 0.0f : sqrt(z1));
    }
    z1 *= -1.0f;

    // Use camera world coordinates whose order is (D,U,R), so the
    // point is (z,y,x).
    Vector4<float> vec1{ z1, mY1, mX1, 0.0f };

    // Create axis and angle for the rotation.
    Vector4<float> axis = Cross(vec0, vec1);
    float dot = Dot(vec0, vec1);
    float angle;
    if (Normalize(axis) > 0.0f)
    {
        angle = acos(std::min(std::max(dot, -1.0f), 1.0f));
    }
    else  // Vectors are parallel.
    {
        if (dot < 0.0f)
        {
            // Rotated pi radians.
            axis[0] = mY0 * invLength0;
            axis[1] = -mX0 * invLength0;
            axis[2] = 0.0f;
            angle = (float)GTE_C_PI;
        }
        else
        {
            // Rotation by zero radians.
            axis.MakeUnit(0);
            angle = 0.0f;
        }
    }

    // Compute the rotation matrix implied by trackball motion.  The axis
    // vector was computed in camera coordinates.  It must be converted
    // to world coordinates.  Once again, I use the camera ordering (D,U,R).
    Vector4<float> worldAxis =
        axis[0] * mCamera->GetDVector() +
        axis[1] * mCamera->GetUVector() +
        axis[2] * mCamera->GetRVector();

    Matrix4x4<float> trackRotate = Rotation<4, float>(
        AxisAngle<4, float>(worldAxis, angle));

    // Compute the new rotation, which is the incremental rotation of
    // the trackball appiled after the object has been rotated by its old
    // rotation.
#if defined(GTE_USE_MAT_VEC)
    Matrix4x4<float> rotate = trackRotate * mInitialOrientation;
#else
    Matrix4x4<float> rotate = mInitialOrientation * trackRotate;
#endif

    // Renormalize to avoid accumulated rounding errors that can cause the
    // rotation matrix to degenerate.
    Vector4<float> v[3] =
    {
        rotate.GetCol(0),
        rotate.GetCol(1),
        rotate.GetCol(2)
    };
    Orthonormalize<4, float>(3, v);
    rotate.SetCol(0, v[0]);
    rotate.SetCol(1, v[1]);
    rotate.SetCol(2, v[2]);

    mRoot->localTransform.SetRotation(rotate);
    mRoot->Update();
}

