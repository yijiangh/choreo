// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#pragma once

#include <Graphics/GteCullingPlane.h>
#include <Graphics/GteTransform.h>

namespace gte
{

class GTE_IMPEXP BoundingSphere
{
public:
    // Construction and destruction.  The default constructor sets the center
    // to the origin (0,0,0,1) and the radius to 0.  A radius of 0 denotes an
    // invalid bound.
    ~BoundingSphere();
    BoundingSphere();
    BoundingSphere(BoundingSphere const& sphere);

    // Assignment.
    BoundingSphere& operator= (BoundingSphere const& sphere);

    // Member access.  The radius must be nonnegative.  When negative, it is
    // clamped to zero.
    inline void SetCenter(Vector4<float> const& center);
    inline void SetRadius(float radius);
    inline Vector4<float> GetCenter() const;
    inline float GetRadius() const;

    // The "positive side" of the plane is the half space to which the
    // plane normal is directed.  The "negative side" is the other half
    // space.  The function returns +1 when the sphere is fully on the
    // positive side, -1 when the sphere is fully on the negative side, or
    // 0 when the sphere is transversely cut by the plane (sphere volume on
    // each side of plane is positive).
    int WhichSide(CullingPlane const& plane) const;

    // Increase 'this' to contain the input sphere.
    void GrowToContain(BoundingSphere const& sphere);

    // Transform the sphere.  If the transform has nonuniform scaling, the
    // resulting object is an ellipsoid.  A sphere is generated to contain
    // the ellipsoid.
    void TransformBy(Transform const& transform,
        BoundingSphere& sphere) const;

    // This function is valid only for 3-channel points (x,y,z) or 4-channel
    // vectors (x,y,z,0) or 4-channel points (x,y,z,1).  In all cases, the
    // function accesses only the (x,y,z) values.  The stride allows you to
    // pass in vertex buffer data.  Set the stride to zero when the points are
    // contiguous in memory.  The 'data' pointer must be to the first point
    // (offset 0).
    void ComputeFromData(int numVertices, int vertexSize, char const* data);

    // Test for intersection of linear component and bound (points of
    // intersection not computed).  The linear component is parameterized by
    // P + t*D, where P is a point on the component (the origin) and D is a
    // unit-length direction vector.  The interval [tmin,tmax] is
    //   line:     tmin = -INFINITY, tmax = INFINITY
    //   ray:      tmin = 0.0f, tmax = INFINITY
    //   segment:  tmin >= 0.0f, tmax > tmin
    // where INFINITY is std::numeric_limits<float>::max().
    bool TestIntersection(Vector4<float> const& origin,
        Vector4<float> const& direction, float tmin, float tmax) const;

    // Test for intersection of the two stationary spheres.
    bool TestIntersection(BoundingSphere const& sphere) const;

    // Test for intersection of the two moving spheres.  The velocity0 is
    // for 'this' and the velocity1 is the for the input bound.
    bool TestIntersection(BoundingSphere const& sphere, float tmax,
        Vector4<float> const& velocity0, Vector4<float> const& velocity1)
        const;

private:
    // (center, radius) = (c0, c1, c2, r)
    Vector4<float> mTuple;
};


inline void BoundingSphere::SetCenter(Vector4<float> const& center)
{
    mTuple[0] = center[0];
    mTuple[1] = center[1];
    mTuple[2] = center[2];
}

inline void BoundingSphere::SetRadius(float radius)
{
    mTuple[3] = (radius >= 0.0f ? radius : 0.0f);
}

inline Vector4<float> BoundingSphere::GetCenter() const
{
    return Vector4<float>{ mTuple[0], mTuple[1], mTuple[2], 1.0f };
}

inline float BoundingSphere::GetRadius() const
{
    return mTuple[3];
}


}
