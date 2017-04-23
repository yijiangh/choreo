// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteBoundingSphere.h>
using namespace gte;


BoundingSphere::~BoundingSphere()
{
}

BoundingSphere::BoundingSphere()
    :
    mTuple({ 0.0f, 0.0f, 0.0f, 0.0f })
{
}

BoundingSphere::BoundingSphere(BoundingSphere const& sphere)
    :
    mTuple(sphere.mTuple)
{
}

BoundingSphere& BoundingSphere::operator= (BoundingSphere const& sphere)
{
    mTuple = sphere.mTuple;
    return *this;
}

int BoundingSphere::WhichSide(CullingPlane const& plane) const
{
    float signedDistance = plane.DistanceTo(GetCenter());
    float radius = GetRadius();

    if (signedDistance <= -radius)
    {
        return -1;
    }

    if (signedDistance >= radius)
    {
        return +1;
    }

    return 0;
}

void BoundingSphere::GrowToContain(BoundingSphere const& sphere)
{
    float radius1 = sphere.GetRadius();
    if (radius1 == 0.0f)
    {
        // The incoming bound is invalid and cannot affect growth.
        return;
    }

    float radius0 = GetRadius();
    if (radius0 == 0.0f)
    {
        // The current bound is invalid, so just assign the incoming bound.
        mTuple = sphere.mTuple;
        return;
    }

    Vector4<float> center0 = GetCenter(), center1 = sphere.GetCenter();
    Vector4<float> centerDiff = center1 - center0;
    float lengthSqr = Dot(centerDiff, centerDiff);
    float radiusDiff = radius1 - radius0;
    float radiusDiffSqr = radiusDiff*radiusDiff;

    if (radiusDiffSqr >= lengthSqr)
    {
        if (radiusDiff >= 0.0f)
        {
            mTuple = sphere.mTuple;
        }
        return;
    }

    float length = sqrt(lengthSqr);
    if (length > 0.0f)
    {
        float coeff = (length + radiusDiff) / (2.0f*length);
        SetCenter(center0 + coeff*centerDiff);
    }

    SetRadius(0.5f*(length + radius0 + radius1));
}

void BoundingSphere::TransformBy(Transform const& transform,
    BoundingSphere& sphere) const
{
#if defined (GTE_USE_MAT_VEC)
    sphere.SetCenter(transform * GetCenter());
#else
    sphere.SetCenter(GetCenter() * transform);
#endif
    sphere.SetRadius(transform.GetNorm() * GetRadius());
}

void BoundingSphere::ComputeFromData(int numVertices, int vertexSize,
    char const* data)
{
    // The center is the average of the positions.
    float sum[3] = { 0.0f, 0.0f, 0.0f };
    int i;
    for (i = 0; i < numVertices; ++i)
    {
        float const* position = (float const*)(data + i*vertexSize);
        sum[0] += position[0];
        sum[1] += position[1];
        sum[2] += position[2];
    }
    float invNumVertices = 1.0f / (float)numVertices;
    mTuple[0] = sum[0] * invNumVertices;
    mTuple[1] = sum[1] * invNumVertices;
    mTuple[2] = sum[2] * invNumVertices;

    // The radius is the largest distance from the center to the positions.
    mTuple[3] = 0.0f;
    for (i = 0; i < numVertices; ++i)
    {
        float const* position = (const float*)(data + i*vertexSize);
        float diff[3] =
        {
            position[0] - mTuple[0],
            position[1] - mTuple[1],
            position[2] - mTuple[2]
        };
        float radiusSqr = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2];
        if (radiusSqr > mTuple[3])
        {
            mTuple[3] = radiusSqr;
        }
    }

    mTuple[3] = sqrt(mTuple[3]);
}

bool BoundingSphere::TestIntersection(Vector4<float> const& origin,
    Vector4<float> const& direction, float tmin, float tmax) const
{
    float radius = GetRadius();
    if (radius == 0.0f)
    {
        // The bound is invalid and cannot be intersected.
        LogWarning(
            "Invalid bound.  Did you forget to call UpdateModelBound()?");
        return false;
    }

    Vector4<float> center = GetCenter();
    float const infinity = std::numeric_limits<float>::max();
    Vector4<float> diff;
    float a0, a1, discr;

    if (tmin == -infinity)
    {
        LogAssert(tmax == infinity, "tmax must be infinity for a line.");

        // Test for sphere-line intersection.
        diff = origin - center;
        a0 = Dot(diff, diff) - radius*radius;
        a1 = Dot(direction, diff);
        discr = a1*a1 - a0;
        return discr >= 0.0f;
    }

    if (tmax == infinity)
    {
        LogAssert(tmin == 0.0f, "tmin must be zero for a ray.");

        // Test for sphere-ray intersection.
        diff = origin - center;
        a0 = Dot(diff, diff) - radius*radius;
        if (a0 <= 0.0f)
        {
            // The ray origin is inside the sphere.
            return true;
        }
        // else: The ray origin is outside the sphere.

        a1 = Dot(direction, diff);
        if (a1 >= 0.0f)
        {
            // The ray forms an acute angle with diff, and so the ray is
            // directed from the sphere.  Thus, the ray origin is outside
            // the sphere, and points P+t*D for t >= 0 are even farther
            // away from the sphere.
            return false;
        }

        discr = a1*a1 - a0;
        return discr >= 0.0f;
    }

    LogAssert(tmax > tmin, "tmin < tmax is required for a segment.");

    // Test for sphere-segment intersection.
    float taverage = 0.5f*(tmin + tmax);
    Vector4<float> segOrigin = origin + taverage * direction;
    float segExtent = 0.5f * (tmax - tmin);

    diff = segOrigin - GetCenter();
    a0 = Dot(diff, diff) - radius*radius;
    if (a0 <= 0.0f)
    {
        // The segment center is inside the sphere.
        return true;
    }

    a1 = Dot(direction, diff);
    discr = a1 * a1 - a0;
    if (discr <= 0.0f)
    {
        // The line is outside the sphere, which implies the segment is also.
        return false;
    }

    // See "3D Game Engine Design (2nd edition)", Section 15.4.3 for the
    // details of the test-intersection query for a segment and a sphere.
    // In the book, 'qval' is the same as '(segment.e - |a1|)^2 - discr'.
    float absA1 = fabs(a1);
    float tmp = segExtent - absA1;
    return tmp * tmp <= discr || segExtent >= absA1;
}

bool BoundingSphere::TestIntersection(BoundingSphere const& sphere) const
{
    if (sphere.GetRadius() == 0.0f || GetRadius() == 0.0f)
    {
        // One of the bounds is invalid and cannot be intersected.
        LogWarning(
            "Invalid bound.  Did you forget to call UpdateModelBound()?");
        return false;
    }

    // Test for staticSphere-staticSphere intersection.
    Vector4<float> diff = GetCenter() - sphere.GetCenter();
    float rSum = GetRadius() + sphere.GetRadius();
    return Dot(diff, diff) <= rSum*rSum;
}

bool BoundingSphere::TestIntersection(BoundingSphere const& sphere,
    float tmax, Vector4<float> const& velocity0,
    Vector4<float> const& velocity1) const
{
    if (sphere.GetRadius() == 0.0f || GetRadius() == 0.0f)
    {
        // One of the bounds is invalid and cannot be intersected.
        LogWarning(
            "Invalid bound.  Did you forget to call UpdateModelBound()?");
        return false;
    }

    // Test for movingSphere-movingSphere intersection.
    Vector4<float> relVelocity = velocity1 - velocity0;
    Vector4<float> cenDiff = sphere.GetCenter() - GetCenter();
    float a = Dot(relVelocity, relVelocity);
    float c = Dot(cenDiff, cenDiff);
    float rSum = sphere.GetRadius() + GetRadius();
    float rSumSqr = rSum*rSum;

    if (a > 0.0f)
    {
        float b = Dot(cenDiff, relVelocity);
        if (b <= 0.0f)
        {
            if (-tmax*a <= b)
            {
                return a*c - b*b <= a*rSumSqr;
            }
            else
            {
                return tmax*(tmax*a + 2.0f*b) + c <= rSumSqr;
            }
        }
    }

    return c <= rSumSqr;
}

