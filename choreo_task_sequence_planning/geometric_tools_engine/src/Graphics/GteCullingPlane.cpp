// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteCullingPlane.h>
using namespace gte;


CullingPlane::~CullingPlane ()
{
}

CullingPlane::CullingPlane ()
    :
    mTuple({ 0.0f, 0.0f, 0.0f, 0.0f })
{
}

CullingPlane::CullingPlane (CullingPlane const& plane)
    :
    mTuple(plane.mTuple)
{
}

CullingPlane::CullingPlane (Vector4<float> const& N, float c)
    :
    mTuple({ N[0], N[1], N[2], c })
{
}

CullingPlane::CullingPlane (float n0, float n1, float n2, float c)
    :
    mTuple({ n0, n1, n2, c })
{
}

CullingPlane::CullingPlane (Vector4<float> const& N, Vector4<float> const& P)
    :
    mTuple({ N[0], N[1], N[2], -Dot(N, P) })
{
}

CullingPlane::CullingPlane (Vector4<float> const& P0,
    Vector4<float> const& P1, Vector4<float> const& P2)
{
    Vector4<float> edge1 = P1 - P0;
    Vector4<float> edge2 = P2 - P0;
    Vector4<float> N = Cross(edge1, edge2);
    mTuple[0] = N[0];
    mTuple[1] = N[1];
    mTuple[2] = N[2];
    mTuple[3] = -Dot(N, P0);
}

CullingPlane& CullingPlane::operator= (CullingPlane const& plane)
{
    mTuple = plane.mTuple;
    return *this;
}

void CullingPlane::Set (Vector4<float> const& N, float c)
{
    mTuple[0] = N[0];
    mTuple[1] = N[1];
    mTuple[2] = N[2];
    mTuple[3] = c;
}

void CullingPlane::Get (Vector4<float>& N, float& c) const
{
    N[0] = mTuple[0];
    N[1] = mTuple[1];
    N[2] = mTuple[2];
    c    = mTuple[3];
}

Vector4<float> CullingPlane::GetNormal () const
{
    return Vector4<float>{ mTuple[0], mTuple[1], mTuple[2], 0.0f };
}

float CullingPlane::GetConstant () const
{
    return mTuple[3];
}

float CullingPlane::Normalize ()
{
    float length = sqrt(mTuple[0]*mTuple[0] + mTuple[1]*mTuple[1] +
        mTuple[2]*mTuple[2]);
    mTuple /= length;
    return length;
}

int CullingPlane::WhichSide (Vector4<float> const& P) const
{
    float distance = Dot(mTuple, P);
    return (distance > 0.0f ? +1 : (distance < 0.0f ? -1 : 0));
}

float CullingPlane::DistanceTo (Vector4<float> const& P) const
{
    return Dot(mTuple, P);
}

