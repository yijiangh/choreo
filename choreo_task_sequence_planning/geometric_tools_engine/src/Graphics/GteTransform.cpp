// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteTransform.h>
#include <algorithm>
using namespace gte;

Transform const Transform::IDENTITY;


Transform::Transform()
    :
    mTranslate({ 0.0f, 0.0f, 0.0f, 1.0f }),
    mScale({ 1.0f, 1.0f, 1.0f, 1.0f }),
    mIsIdentity(true),
    mIsRSMatrix(true),
    mIsUniformScale(true),
    mInverseNeedsUpdate(false)
{
    mHMatrix.MakeIdentity();
    mInvHMatrix.MakeIdentity();
    mMatrix.MakeIdentity();
}

void Transform::MakeIdentity()
{
    mMatrix.MakeIdentity();
    mTranslate = { 0.0f, 0.0f, 0.0f, 1.0f };
    mScale = { 1.0f, 1.0f, 1.0f, 1.0f };
    mIsIdentity = true;
    mIsRSMatrix = true;
    mIsUniformScale = true;
    UpdateHMatrix();
}

void Transform::MakeUnitScale()
{
    LogAssert(mIsRSMatrix, "Transform is not rotation-scale.");
    mScale = { 1.0f, 1.0f, 1.0f, 1.0f };
    mIsUniformScale = true;
    UpdateHMatrix();
}

void Transform::SetRotation(Matrix4x4<float> const& rotate)
{
    mMatrix = rotate;
    mIsIdentity = false;
    mIsRSMatrix = true;
    UpdateHMatrix();
}

void Transform::SetMatrix(Matrix4x4<float> const& matrix)
{
    mMatrix = matrix;
    mIsIdentity = false;
    mIsRSMatrix = false;
    mIsUniformScale = false;
    UpdateHMatrix();
}

void Transform::SetTranslation(float x0, float x1, float x2)
{
    mTranslate = { x0, x1, x2, 1.0f };
    mIsIdentity = false;
    UpdateHMatrix();
}

void Transform::SetTranslation(Vector3<float> const& translate)
{
    SetTranslation(translate[0], translate[1], translate[2]);
}

void Transform::SetTranslation(Vector4<float> const& translate)
{
    SetTranslation(translate[0], translate[1], translate[2]);
}

void Transform::SetScale(float s0, float s1, float s2)
{
    LogAssert(mIsRSMatrix, "Transform is not rotation-scale.");
    LogAssert(s0 != 0.0f && s1 != 0.0f && s2 != 0.0f,
        "Scales must be nonzero.");

    mScale = { s0, s1, s2, 1.0f };
    mIsIdentity = false;
    mIsUniformScale = false;
    UpdateHMatrix();
}

void Transform::SetScale(Vector3<float> const& scale)
{
    SetScale(scale[0], scale[1], scale[2]);
}

void Transform::SetScale(Vector4<float> const& scale)
{
    SetScale(scale[0], scale[1], scale[2]);
}

void Transform::SetUniformScale(float scale)
{
    LogAssert(mIsRSMatrix, "Transform is not rotation-scale.");
    LogAssert(scale != 0.0f, "Scale must be nonzero.");

    mScale = { scale, scale, scale, 1.0f };
    mIsIdentity = false;
    mIsUniformScale = true;
    UpdateHMatrix();
}

void Transform::SetRotation(Matrix3x3<float> const& rotate)
{
    mMatrix.MakeIdentity();
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            mMatrix(r, c) = rotate(r, c);
        }
    }

    mIsIdentity = false;
    mIsRSMatrix = true;
    UpdateHMatrix();
}

void Transform::GetRotation(Matrix3x3<float>& rotate) const
{
    LogAssert(mIsRSMatrix, "Transform is not rotation-scale.");
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            rotate(r, c) = mMatrix(r, c);
        }
    }
}

void Transform::SetRotation(Quaternion<float> const& q)
{
    mMatrix = Rotation<4, float>(q);
    mIsIdentity = false;
    mIsRSMatrix = true;
    UpdateHMatrix();
}

void Transform::GetRotation(Quaternion<float>& q) const
{
    LogAssert(mIsRSMatrix, "Transform is not rotation-scale.");
    q = Rotation<4, float>(mMatrix);
}

void Transform::SetRotation(AxisAngle<4, float> const& axisAngle)
{
    mMatrix = Rotation<4, float>(axisAngle);
    mIsIdentity = false;
    mIsRSMatrix = true;
    UpdateHMatrix();
}

void Transform::GetRotation(AxisAngle<4, float>& axisAngle) const
{
    LogAssert(mIsRSMatrix, "Transform is not rotation-scale.");
    axisAngle = Rotation<4, float>(mMatrix);
}

void Transform::SetRotation(EulerAngles<float> const& eulerAngles)
{
    mMatrix = Rotation<4, float>(eulerAngles);
    mIsIdentity = false;
    mIsRSMatrix = true;
    UpdateHMatrix();
}

void Transform::GetRotation(EulerAngles<float>& eulerAngles) const
{
    LogAssert(mIsRSMatrix, "Transform is not rotation-scale.");
    eulerAngles = Rotation<4, float>(mMatrix)(eulerAngles.axis[0],
        eulerAngles.axis[1], eulerAngles.axis[2]);
}

float Transform::GetNorm() const
{
    float r0, r1, r2;

    if (mIsRSMatrix)
    {
        // A RS matrix (GTE_USE_MAT_VEC) or an SR matrix (GTE_USE_VEC_MAT).
        r0 = fabs(mScale[0]);
        r1 = fabs(mScale[1]);
        r2 = fabs(mScale[2]);
    }
    else
    {
        // The spectral norm (the maximum absolute value of the eigenvalues)
        // is smaller or equal to this norm.  Therefore, this function returns
        // an approximation to the maximum scale.

#if defined(GTE_USE_MAT_VEC)
        // Use the max-row-sum matrix norm.
        r0 = fabs(mMatrix(0, 0)) + fabs(mMatrix(0, 1)) + fabs(mMatrix(0, 2));
        r1 = fabs(mMatrix(1, 0)) + fabs(mMatrix(1, 1)) + fabs(mMatrix(1, 2));
        r2 = fabs(mMatrix(2, 0)) + fabs(mMatrix(2, 1)) + fabs(mMatrix(2, 2));
#else
        // Use the max-col-sum matrix norm.
        r0 = fabs(mMatrix(0, 0)) + fabs(mMatrix(1, 0)) + fabs(mMatrix(2, 0));
        r1 = fabs(mMatrix(0, 1)) + fabs(mMatrix(1, 1)) + fabs(mMatrix(2, 1));
        r2 = fabs(mMatrix(0, 2)) + fabs(mMatrix(1, 2)) + fabs(mMatrix(2, 2));
#endif
    }

    return std::max(std::max(r0, r1), r2);
}

Matrix4x4<float> const& Transform::GetHInverse() const
{
    if (mInverseNeedsUpdate)
    {
        if (mIsIdentity)
        {
            mInvHMatrix.MakeIdentity();
        }
        else
        {
            if (mIsRSMatrix)
            {
                if (mIsUniformScale)
                {
                    float invScale = 1.0f / mScale[0];
#if defined(GTE_USE_MAT_VEC)
                    mInvHMatrix(0, 0) = invScale * mMatrix(0, 0);
                    mInvHMatrix(0, 1) = invScale * mMatrix(1, 0);
                    mInvHMatrix(0, 2) = invScale * mMatrix(2, 0);
                    mInvHMatrix(1, 0) = invScale * mMatrix(0, 1);
                    mInvHMatrix(1, 1) = invScale * mMatrix(1, 1);
                    mInvHMatrix(1, 2) = invScale * mMatrix(2, 1);
                    mInvHMatrix(2, 0) = invScale * mMatrix(0, 2);
                    mInvHMatrix(2, 1) = invScale * mMatrix(1, 2);
                    mInvHMatrix(2, 2) = invScale * mMatrix(2, 2);
#else
                    mInvHMatrix(0, 0) = mMatrix(0, 0) * invScale;
                    mInvHMatrix(0, 1) = mMatrix(1, 0) * invScale;
                    mInvHMatrix(0, 2) = mMatrix(2, 0) * invScale;
                    mInvHMatrix(1, 0) = mMatrix(0, 1) * invScale;
                    mInvHMatrix(1, 1) = mMatrix(1, 1) * invScale;
                    mInvHMatrix(1, 2) = mMatrix(2, 1) * invScale;
                    mInvHMatrix(2, 0) = mMatrix(0, 2) * invScale;
                    mInvHMatrix(2, 1) = mMatrix(1, 2) * invScale;
                    mInvHMatrix(2, 2) = mMatrix(2, 2) * invScale;
#endif
                }
                else
                {
                    // Replace 3 reciprocals by 6 multiplies and 1 reciprocal.
                    float s01 = mScale[0] * mScale[1];
                    float s02 = mScale[0] * mScale[2];
                    float s12 = mScale[1] * mScale[2];
                    float invs012 = 1.0f / (s01 * mScale[2]);
                    float invS0 = s12 * invs012;
                    float invS1 = s02 * invs012;
                    float invS2 = s01 * invs012;
#if defined(GTE_USE_MAT_VEC)
                    mInvHMatrix(0, 0) = invS0 * mMatrix(0, 0);
                    mInvHMatrix(0, 1) = invS0 * mMatrix(1, 0);
                    mInvHMatrix(0, 2) = invS0 * mMatrix(2, 0);
                    mInvHMatrix(1, 0) = invS1 * mMatrix(0, 1);
                    mInvHMatrix(1, 1) = invS1 * mMatrix(1, 1);
                    mInvHMatrix(1, 2) = invS1 * mMatrix(2, 1);
                    mInvHMatrix(2, 0) = invS2 * mMatrix(0, 2);
                    mInvHMatrix(2, 1) = invS2 * mMatrix(1, 2);
                    mInvHMatrix(2, 2) = invS2 * mMatrix(2, 2);
#else
                    mInvHMatrix(0, 0) = mMatrix(0, 0) * invS0;
                    mInvHMatrix(0, 1) = mMatrix(1, 0) * invS1;
                    mInvHMatrix(0, 2) = mMatrix(2, 0) * invS2;
                    mInvHMatrix(1, 0) = mMatrix(0, 1) * invS0;
                    mInvHMatrix(1, 1) = mMatrix(1, 1) * invS1;
                    mInvHMatrix(1, 2) = mMatrix(2, 1) * invS2;
                    mInvHMatrix(2, 0) = mMatrix(0, 2) * invS0;
                    mInvHMatrix(2, 1) = mMatrix(1, 2) * invS1;
                    mInvHMatrix(2, 2) = mMatrix(2, 2) * invS2;
#endif
                }
            }
            else
            {
                Invert3x3(mHMatrix, mInvHMatrix);
            }

#if defined(GTE_USE_MAT_VEC)
            mInvHMatrix(0, 3) = -(
                mInvHMatrix(0, 0) * mTranslate[0] +
                mInvHMatrix(0, 1) * mTranslate[1] +
                mInvHMatrix(0, 2) * mTranslate[2]
                );

            mInvHMatrix(1, 3) = -(
                mInvHMatrix(1, 0) * mTranslate[0] +
                mInvHMatrix(1, 1) * mTranslate[1] +
                mInvHMatrix(1, 2) * mTranslate[2]
                );

            mInvHMatrix(2, 3) = -(
                mInvHMatrix(2, 0) * mTranslate[0] +
                mInvHMatrix(2, 1) * mTranslate[1] +
                mInvHMatrix(2, 2) * mTranslate[2]
                );

            // The last row of mHMatrix is always (0,0,0,1) for an affine
            // transformation, so it is set once in the constructor.  It is
            // not necessary to reset it here.
#else
            mInvHMatrix(3, 0) = -(
                mInvHMatrix(0, 0) * mTranslate[0] +
                mInvHMatrix(1, 0) * mTranslate[1] +
                mInvHMatrix(2, 0) * mTranslate[2]
                );

            mInvHMatrix(3, 1) = -(
                mInvHMatrix(0, 1) * mTranslate[0] +
                mInvHMatrix(1, 1) * mTranslate[1] +
                mInvHMatrix(2, 1) * mTranslate[2]
                );

            mInvHMatrix(3, 2) = -(
                mInvHMatrix(0, 2) * mTranslate[0] +
                mInvHMatrix(1, 2) * mTranslate[1] +
                mInvHMatrix(2, 2) * mTranslate[2]
                );

            // The last column of mHMatrix is always (0,0,0,1) for an affine
            // transformation, so it is set once in the constructor.  It is
            // not necessary to reset it here.
#endif
        }

        mInverseNeedsUpdate = false;
    }

    return mInvHMatrix;
}

Transform Transform::Inverse() const
{
    Transform inverse;

    if (mIsIdentity)
    {
        inverse.MakeIdentity();
    }
    else
    {
        if (mIsRSMatrix && mIsUniformScale)
        {
            inverse.SetRotation(Transpose(GetRotation()));
            inverse.SetUniformScale(1.0f / GetUniformScale());
        }
        else
        {
            Matrix4x4<float> invUpper;
            Invert3x3(GetMatrix(), invUpper);
            inverse.SetMatrix(invUpper);
        }
        Vector4<float> trn = -GetTranslationW0();
        inverse.SetTranslation(inverse.GetMatrix() * trn);
    }

    mInverseNeedsUpdate = true;
    return inverse;
}

void Transform::UpdateHMatrix()
{
    if (mIsIdentity)
    {
        mHMatrix.MakeIdentity();
    }
    else
    {
        if (mIsRSMatrix)
        {
#if defined(GTE_USE_MAT_VEC)
            mHMatrix(0, 0) = mMatrix(0, 0) * mScale[0];
            mHMatrix(0, 1) = mMatrix(0, 1) * mScale[1];
            mHMatrix(0, 2) = mMatrix(0, 2) * mScale[2];
            mHMatrix(1, 0) = mMatrix(1, 0) * mScale[0];
            mHMatrix(1, 1) = mMatrix(1, 1) * mScale[1];
            mHMatrix(1, 2) = mMatrix(1, 2) * mScale[2];
            mHMatrix(2, 0) = mMatrix(2, 0) * mScale[0];
            mHMatrix(2, 1) = mMatrix(2, 1) * mScale[1];
            mHMatrix(2, 2) = mMatrix(2, 2) * mScale[2];
#else
            mHMatrix(0, 0) = mScale[0] * mMatrix(0, 0);
            mHMatrix(0, 1) = mScale[0] * mMatrix(0, 1);
            mHMatrix(0, 2) = mScale[0] * mMatrix(0, 2);
            mHMatrix(1, 0) = mScale[1] * mMatrix(1, 0);
            mHMatrix(1, 1) = mScale[1] * mMatrix(1, 1);
            mHMatrix(1, 2) = mScale[1] * mMatrix(1, 2);
            mHMatrix(2, 0) = mScale[2] * mMatrix(2, 0);
            mHMatrix(2, 1) = mScale[2] * mMatrix(2, 1);
            mHMatrix(2, 2) = mScale[2] * mMatrix(2, 2);
#endif
        }
        else
        {
            mHMatrix(0, 0) = mMatrix(0, 0);
            mHMatrix(0, 1) = mMatrix(0, 1);
            mHMatrix(0, 2) = mMatrix(0, 2);
            mHMatrix(1, 0) = mMatrix(1, 0);
            mHMatrix(1, 1) = mMatrix(1, 1);
            mHMatrix(1, 2) = mMatrix(1, 2);
            mHMatrix(2, 0) = mMatrix(2, 0);
            mHMatrix(2, 1) = mMatrix(2, 1);
            mHMatrix(2, 2) = mMatrix(2, 2);
        }

#if defined(GTE_USE_MAT_VEC)
        mHMatrix(0, 3) = mTranslate[0];
        mHMatrix(1, 3) = mTranslate[1];
        mHMatrix(2, 3) = mTranslate[2];

        // The last row of mHMatrix is always (0,0,0,1) for an affine
        // transformation, so it is set once in the constructor.  It is not
        // necessary to reset it here.
#else
        mHMatrix(3, 0) = mTranslate[0];
        mHMatrix(3, 1) = mTranslate[1];
        mHMatrix(3, 2) = mTranslate[2];

        // The last column of mHMatrix is always (0,0,0,1) for an affine
        // transformation, so it is set once in the constructor.  It is not
        // necessary to reset it here.
#endif
    }

    mInverseNeedsUpdate = true;
}

void Transform::Invert3x3(Matrix4x4<float> const& mat,
    Matrix4x4<float>& invMat)
{
    // Compute the adjoint of M (3x3).
    invMat(0, 0) = mat(1, 1) * mat(2, 2) - mat(1, 2) * mat(2, 1);
    invMat(0, 1) = mat(0, 2) * mat(2, 1) - mat(0, 1) * mat(2, 2);
    invMat(0, 2) = mat(0, 1) * mat(1, 2) - mat(0, 2) * mat(1, 1);
    invMat(0, 3) = 0.0f;
    invMat(1, 0) = mat(1, 2) * mat(2, 0) - mat(1, 0) * mat(2, 2);
    invMat(1, 1) = mat(0, 0) * mat(2, 2) - mat(0, 2) * mat(2, 0);
    invMat(1, 2) = mat(0, 2) * mat(1, 0) - mat(0, 0) * mat(1, 2);
    invMat(1, 3) = 0.0f;
    invMat(2, 0) = mat(1, 0) * mat(2, 1) - mat(1, 1) * mat(2, 0);
    invMat(2, 1) = mat(0, 1) * mat(2, 0) - mat(0, 0) * mat(2, 1);
    invMat(2, 2) = mat(0, 0) * mat(1, 1) - mat(0, 1) * mat(1, 0);
    invMat(2, 3) = 0.0f;
    invMat(3, 0) = 0.0f;
    invMat(3, 1) = 0.0f;
    invMat(3, 2) = 0.0f;
    invMat(3, 3) = 1.0f;

    // Compute the reciprocal of the determinant of M.
    float invDet = 1.0f / (
        mat(0, 0) * invMat(0, 0) +
        mat(0, 1) * invMat(1, 0) +
        mat(0, 2) * invMat(2, 0)
        );

    // inverse(M) = adjoint(M)/determinant(M).
    invMat(0, 0) *= invDet;
    invMat(0, 1) *= invDet;
    invMat(0, 2) *= invDet;
    invMat(1, 0) *= invDet;
    invMat(1, 1) *= invDet;
    invMat(1, 2) *= invDet;
    invMat(2, 0) *= invDet;
    invMat(2, 1) *= invDet;
    invMat(2, 2) *= invDet;
}

namespace gte
{

Vector4<float> operator*(Transform const& M, Vector4<float> const& V)
{
    return M.GetHMatrix() * V;
}

Vector4<float> operator*(Vector4<float> const& V, Transform const& M)
{
    return V * M.GetHMatrix();
}

Transform operator*(Transform const& A, Transform const& B)
{
    if (A.IsIdentity())
    {
        return B;
    }

    if (B.IsIdentity())
    {
        return A;
    }

    Transform product;

    if (A.IsRSMatrix() && B.IsRSMatrix())
    {
#if defined(GTE_USE_MAT_VEC)
        if (A.IsUniformScale())
        {
            product.SetRotation(A.GetRotation() * B.GetRotation());

            product.SetTranslation(A.GetUniformScale()*(
                A.GetRotation() * B.GetTranslationW0()) +
                A.GetTranslationW1());

            if (B.IsUniformScale())
            {
                product.SetUniformScale(
                    A.GetUniformScale() * B.GetUniformScale());
            }
            else
            {
                product.SetScale(A.GetUniformScale() * B.GetScale());
            }

            return product;
        }
#else
        if (B.IsUniformScale())
        {
            product.SetRotation(A.GetRotation() * B.GetRotation());

            product.SetTranslation(B.GetUniformScale()*(
                A.GetTranslationW0() * B.GetRotation()) + B.GetTranslationW1());

            if (A.IsUniformScale())
            {
                product.SetUniformScale(
                    A.GetUniformScale() * B.GetUniformScale());
            }
            else
            {
                product.SetScale(A.GetScale() * B.GetUniformScale());
            }

            return product;
        }
#endif
    }

    // In all remaining cases, the matrix cannot be written as R*S*X+T.
    Matrix4x4<float> matMA;
    if (A.IsRSMatrix())
    {
#if defined(GTE_USE_MAT_VEC)
        matMA = MultiplyMD(A.GetRotation(), A.GetScaleW1());
#else
        matMA = MultiplyDM(A.GetScaleW1(), A.GetRotation());
#endif
    }
    else
    {
        matMA = A.GetMatrix();
    }

    Matrix4x4<float> matMB;
    if (B.IsRSMatrix())
    {
#if defined(GTE_USE_MAT_VEC)
        matMB = MultiplyMD(B.GetRotation(), B.GetScaleW1());
#else
        matMB = MultiplyDM(B.GetScaleW1(), B.GetRotation());
#endif
    }
    else
    {
        matMB = B.GetMatrix();
    }

    product.SetMatrix(matMA * matMB);
#if defined(GTE_USE_MAT_VEC)
    product.SetTranslation(matMA * B.GetTranslationW0() +
        A.GetTranslationW1());
#else
    product.SetTranslation(A.GetTranslationW0() * matMB +
        B.GetTranslationW1());
#endif
    return product;
}

}
