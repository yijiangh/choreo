// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#pragma once

#include <Mathematics/GteVector4.h>

// The plane is defined by Dot((n0,n1,n2,c),(x0,x1,x2,1)) = 0, where a
// plane normal is N = (n0,n1,n2,0), c is a constant, and X = (x0,x1,x2,1)
// is a point on the plane.  If P = (p0,p1,p2,1) is a point on the plane, then
// Dot(N,X-P) = 0, in which case c = -Dot(N,P).  If P0, P1, and P2 are points
// on the plane and are not collinear, then N = Cross(P1-P0,P2-P0) and
// c = -Dot(N,P0).

namespace gte
{

class GTE_IMPEXP CullingPlane
{
public:
    // Construction and destruction.  The destructor hides the base-class
    // destructor, but the latter has no side effects.
    ~CullingPlane ();
    CullingPlane ();
    CullingPlane (CullingPlane const& plane);
    CullingPlane (Vector4<float> const& N, float c);
    CullingPlane (float n0, float n1, float n2, float c);
    CullingPlane (Vector4<float> const& N, Vector4<float> const& P);
    CullingPlane (Vector4<float> const& P0, Vector4<float> const& P1,
        Vector4<float> const& P2);

    // Assignment.
    CullingPlane& operator= (CullingPlane const& plane);

    // Member access.  Because N and c are interdependent, there are no
    // accessors to set N or c individually.
    void Set (Vector4<float> const& N, float c);
    void Get (Vector4<float>& N, float& c) const;
    Vector4<float> GetNormal () const;
    float GetConstant () const;

    // Compute L = Length(n0,n1,n2) and set the plane to (n0,n1,n2,c)/L.
    // This is useful when transforming planes by homogeneous matrices.
    // The function returns L.
    float Normalize ();

    // The "positive side" of the plane is the half space to which the
    // plane normal is directed.  The "negative side" is the other half
    // space.  The function returns +1 when P is on the positive side, -1
    // when P is on the negative side, or 0 when P is on the plane.
    int WhichSide (Vector4<float> const& P) const;

    // Compute d = Dot(N,P)+c where N is the plane normal and c is the plane
    // constant.  This is a signed pseudodistance.  The sign of the return
    // value is consistent with that in the comments for WhichSide(...).
    float DistanceTo (Vector4<float> const& P) const;

private:
    Vector4<float> mTuple;
};

}
