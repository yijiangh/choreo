// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.6.0 (2017/01/02)

#pragma once

#include <Mathematics/GteQuaternion.h>

// A quaternion is of the form
//   q = w + x * i + y * j + z * k
// where w, x, y, and z are real numbers.  The scalar and vector parts are
//   Scalar(q) = w
//   Vector(q) = x * i + y * j + z * k
// I assume that you are familiar with the arithmetic and algebraic properties
// of quaternions.  See
// https://www.geometrictools.com/Documentation/Quaternions.pdf
//
// A dual number (or dual scalar) is of the form
//   v = a + b * e
// where e is a symbol representing a nonzero quantity but e*e = 0 and where
// (for the purposes of this code) a and b are real numbers.  Addition,
// subtraction, and scalar multiplication are performed componentwise,
//   (a0 + b0 * e) + (a1 + b1 * e) = (a0 + a1) + (b0 + b1) * e
//   (a0 + b0 * e) - (a1 + b1 * e) = (a0 - a1) + (b0 - b1) * e
//   c * (a + b * e) = (c * a) + (c * b) * e
// where c is a real number.  Multiplication is
//   (a0 + b0 * e) * (a1 + b1 * e) = (a0 * a1) + (a0 * b1 + b0 * a1) * e
// Function evaluation (for a differentiable function F) is
//   F(a + b * e) = F(a) + [b * F'(a)] * e
// where F' is the derivative of F.  This property is used for "automatic
// differentiation" of functions.
//
// A dual quaternion is of the form
//   d = (w0+w1*e) + (x0+x1*e) * i + (y0+y1*e) * j + (z0+z1*e) * k
// where the coefficients are dual numbers.  This has the same appearance as
// quaternions except that the real-valued coefficients of the quaternions are
// replace by dual-number-valued coefficients.  Define the quaternions
// p0 = w0 + x0 * i + y0 * j + z0 * k and p1 = w1 + x1 * i + y1 * j + z1 * k;
// then d = p0 + p1 * e.  By convention, the abstract symbol e commutes with
// i, j, and k.
//
// Define the dual quaternions
//   A = a0 + a1*i + a2*j + a3*k = pa + qa * e
//   B = b0 + b1*i + b2*j + b3*k = pb + qb * e
//   C = c0 + c1*i + c2*j + c3*k = pc + qc * e
// where all coefficients are dual numbers.
//
// Scalar and vector parts.
//   Scalar(A) = Scalar(pa) + Scalar(qa) * e
//   Vector(A) = Vector(pa) + Vector(qa) * e
//   A = Scalar(A) + Vector(A)
//
// Addition, subtraction, and scalar multiplication of dual quaternions are
// symbolically the same as for quaternions.  For example,
//   A + B = (a0 + b0) + (a1 + b1) * i + (a2 + b2) * j + (a3 + b3) * k
// NOTE:  These operations are not implemented here, because the goal of the
// DualQuaternion class is to support representation of rigid transformations.
//
// Multiplication (noncommutative):
//   A * B = (pa * pb) + (pa * qb + qa * pb) * e
//         =   (a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3) * 1
//           + (a1 * b0 + a0 * b1 - a3 * b2 + a2 * b3) * i
//           + (a2 * b0 + a3 * b1 + a0 * b2 - a1 * b3) * j
//         = + (a3 * b0 - a2 * b1 + a1 * b2 - a0 * b3) * k
// which is the same symbolic formula for the multiplication of quaternions
// but with the real-valued coefficients replaced by the dual-number-valued
// coefficients.  As with quaternions, it is not generally true that
// B * A = A * AB.
//
// Conjugate operation:
//   conj(A) = conj(pa) + conj(qa) * e = Scalar(A) - Vector(A)
//
// Scalar product of dual quaternions:
//   dot(A,B) = a0 * b0 + a1 * b1 + a2 * b2 + a3 * b3
//            = (A * conj(B) + B * conj(A)) / 2
//            = dot(B,A)
//
// Vector product of dual quaternions:
//   cross(A,B) = (A * B - B * A) / 2
//
// Norm (squared magnitude):
//   Norm(A) = A * conj(A) = conj(A) * A = dot(A, A)
//           = (pa * conj(pa)) + (pa * conj(qa) + qa * conj(pa)) * e
// Note that this is a dual scalar of the form s0 + s1 * e, where s0 and s1
// are scalars.
//
// Function of a dual quaternion, which requires function F to be
// differentiable
//   F(A) = F(pa + qa * e) = F(pa) + (qa * F'(pa)) * e
// This property is used in "automatic differentiation."
//
// Length (magnitude):  The squared magnitude is Norm(A) = s0 + s1 * e,
// so we must compute the square root of this.
//   Length(A) = sqrt(Norm(A)) = sqrt(s0) + (s1 / (2 * sqrt(s0))) * e
//
// A unit dual quaternion U = p + q * e has the property Norm(U) = 1.  This
// happens when
//   p * conj(p) = 1 [p is a unit quaternion] and
//   p * conj(q) + q * conj(p) = 0
//
// For a dual quaternion A = p + q * e where p is not zero, the inverse is
//   Inverse(A) = Inverse(p) - Inverse(p) * q * Inverse(p) * e
// Note that dual quaternions of the form 0 + q * e are not invertible.
//
// Application to rigid transformations in 3D, Y = R * X + T, where
// R is a 3x3 rotation matrix, T is a 3x1 translation vector, X is the
// 3x1 input, and Y is the 3x1 output.
//
// The rigid transformation is represented by the dual quaternion
//   d = r + (t * r / 2) * e
// where r is a unit quaternion representing the rotation and t is a
// quaternion of the form t = 0 + T0 * i + T1 * j + T2 * k, where the
// translation as a 3-tuple is T = (T0,T1,T2).  Given a dual quaternion,
// the quaternion for the rotation is the scalar part r.  To extract the
// translation, the vector part is processed as follows,
//   t = 2 * (t * r / 2) * conj(r)
//
// The product of rigid transformations d0 = r0 + (t0 * r0 / 2) * e and
// d1 = r1 + (t1 * r1 / 2) is
//   d0 * d1 = (r0 * r1) + ((r0 * t1 * r1 + t0 * r0 * r1) / 2) * e
// The quaternion for rotation is r0 * r1, as expected.  The translation t'
// is extracted as
//   t' = 2 * ((r0 * t1 * r1 + t0 * r0 * r1) / 2) * conj(r0 * r1)
//      = (r0 * t1 * r1 + t0 * r0 * r1) * (conj(r1) * conj(r0))
//      = r0 * t1 * conj(r0) + t0
// This is consistent with the product of 4x4 affine matrices
//   +-     -+ +-     -+   +-              -+
//   | R0 T0 | | R1 T1 | = | R0*R1 R0*T1+T0 |
//   | 0  1  | | 0  1  |   | 0     1        |
//   +-     -+ +-     -+   +-              -+
// The right-hand side is a rotation R0*R1, represented by quaternion
// r0*r1, followed by a translation R0*T1+T0, represented by quaternion
// r0*t1*conj(r0)+t0.  The first term of this expression is a rotation
// of t1 by r0.  The second term add in the tranlation t0.
//
// To apply a dual quaternion representing a rigid transformation to a
// 3-tuple point X = (X0,X1,X2) represented as a quaternion
// x = 0 + X0 * i + X1 * j + X2 * k, consider
//   +-   -+ +-   -+   +-       -+
//   | R T | | I X | = | R R*X+T |
//   | 0 1 | | 0 1 |   | 0 1     |
//   +-   -+ +-   -+   +-       -+
// The rigid transformation is the translational component of the
// matrix, R*X+T = Y.  The second matrix of the left-hand side can be
// represented as a dual quaternion 1+(x/2)*e, so the product on
// the left-hand side is
//   [r + (t * r / 2) * e] * [1 + (x / 2) * e]
//   = r + ((r * x + t * r) / 2) * e
// Extract the translation component by
//   y = 2 * ((r * x + t * r) / 2) * conj(r)
//     = r * x * conj(r) + t
// The first term of the right-hand side is the quaternion formulation
// for rotating X by R.  The second term is the addition of the
// translation vector T.
//
// TODO: Interpolation of rigid transformations (using the geodesic
// idea similar to quaternions).

namespace gte
{

template <typename Real>
class DualQuaternion
{
public:
    // Construction.  The default constructor creates the identiy dual
    // quaternion 1+0*e.
    DualQuaternion();

    DualQuaternion(DualQuaternion const& d);

    // Create the dual quaternion p+q*e.
    DualQuaternion(Quaternion<Real> const& p, Quaternion<Real> const& q);

    // Assignment.
    DualQuaternion& operator=(DualQuaternion const& d);

    // Member access.
    Quaternion<Real> const& operator[] (int i) const;
    Quaternion<Real>& operator[] (int i);

    // Special quaternions.
    static DualQuaternion Zero();   // 0+0*e
    static DualQuaternion Identity();   // 1+0*e

private:
    // The dual quaternion is mTuple[0] + mTuple[1] * e.
    std::array<Quaternion<Real>, 2> mTuple;
};


// Unary operations.
template <typename Real>
DualQuaternion<Real> operator+(DualQuaternion<Real> const& d);

template <typename Real>
DualQuaternion<Real> operator-(DualQuaternion<Real> const& d);

// Linear-algebraic operations.
template <typename Real>
DualQuaternion<Real> operator+(DualQuaternion<Real> const& d0, DualQuaternion<Real> const& d1);

template <typename Real>
DualQuaternion<Real> operator-(DualQuaternion<Real> const& d0, DualQuaternion<Real> const& d1);

template <typename Real>
DualQuaternion<Real> operator*(DualQuaternion<Real> const& d, Real scalar);

template <typename Real>
DualQuaternion<Real> operator*(Real scalar, DualQuaternion<Real> const& d);

template <typename Real>
DualQuaternion<Real> operator/(DualQuaternion<Real> const& d, Real scalar);

template <typename Real>
DualQuaternion<Real>& operator+=(DualQuaternion<Real>& d0, DualQuaternion<Real> const& d1);

template <typename Real>
DualQuaternion<Real>& operator-=(DualQuaternion<Real>& d0, DualQuaternion<Real> const& d1);

template <typename Real>
DualQuaternion<Real>& operator*=(DualQuaternion<Real>& d, Real scalar);

template <typename Real>
DualQuaternion<Real>& operator/=(DualQuaternion<Real>& d, Real scalar);

// Multiplication of dual quaternions.
template <typename Real>
DualQuaternion<Real> operator*(DualQuaternion<Real> const& d0, DualQuaternion<Real> const& d1);

// The conjugate of a dual quaternion
template <typename Real>
DualQuaternion<Real> Conjugate(DualQuaternion<Real> const& d);

// Inverse of a dual quaternion p+q*e for which p is not zero.
// If p is zero, the function returns the zero dual quaternion
// as a signal the inversion failed.
template <typename Real>
Quaternion<Real> Inverse(Quaternion<Real> const& d);

// Geometric operations.  See GteVector.h for a description of the
// 'robust' parameter.
template <int N, typename Real>
DualQuaternion<Real> Dot(DualQuaternion<Real> const& d0, DualQuaternion<Real> const& d1);

template <int N, typename Real>
DualQuaternion<Real> Cross(DualQuaternion<Real> const& d0, DualQuaternion<Real> const& d1);

template <int N, typename Real>
DualQuaternion<Real> Norm(DualQuaternion<Real>& d, bool robust = false);

template <int N, typename Real>
DualQuaternion<Real> Length(DualQuaternion<Real> const& d, bool robust = false);

// Transform a vector X by the rigid transformation Y = R * X + T represented
// by the input dual quaternion.  X and Y are stored in point format (4-tuples
// whose w-components are 1).
template <typename Real>
Vector<4, Real> RigidTransform(DualQuaternion<Real> const& d, Vector<4, Real> const& v);

#if 0
template <typename Real>
Quaternion<Real>::Quaternion()
{
    // Uninitialized.
}

template <typename Real>
Quaternion<Real>::Quaternion(Quaternion const& q)
{
    this->mTuple[0] = q[0];
    this->mTuple[1] = q[1];
    this->mTuple[2] = q[2];
    this->mTuple[3] = q[3];
}

template <typename Real>
Quaternion<Real>::Quaternion(Vector<4, Real> const& q)
{
    this->mTuple[0] = q[0];
    this->mTuple[1] = q[1];
    this->mTuple[2] = q[2];
    this->mTuple[3] = q[3];
}

template <typename Real>
Quaternion<Real>::Quaternion(Real x, Real y, Real z, Real w)
{
    this->mTuple[0] = x;
    this->mTuple[1] = y;
    this->mTuple[2] = z;
    this->mTuple[3] = w;
}

template <typename Real>
Quaternion<Real>& Quaternion<Real>::operator=(Quaternion const& q)
{
    Vector<4, Real>::operator=(q);
    return *this;
}

template <typename Real>
Quaternion<Real>& Quaternion<Real>::operator=(Vector<4, Real> const& q)
{
    Vector<4, Real>::operator=(q);
    return *this;
}

template <typename Real>
Quaternion<Real> Quaternion<Real>::Zero()
{
    return Quaternion((Real)0, (Real)0, (Real)0, (Real)0);
}

template <typename Real>
Quaternion<Real> Quaternion<Real>::I()
{
    return Quaternion((Real)1, (Real)0, (Real)0, (Real)0);
}

template <typename Real>
Quaternion<Real> Quaternion<Real>::J()
{
    return Quaternion((Real)0, (Real)1, (Real)0, (Real)0);
}

template <typename Real>
Quaternion<Real> Quaternion<Real>::K()
{
    return Quaternion((Real)0, (Real)0, (Real)1, (Real)0);
}

template <typename Real>
Quaternion<Real> Quaternion<Real>::Identity()
{
    return Quaternion((Real)0, (Real)0, (Real)0, (Real)1);
}

template <typename Real>
Quaternion<Real> operator*(Quaternion<Real> const& q0,
    Quaternion<Real> const& q1)
{
    // (x0*i + y0*j + z0*k + w0)*(x1*i + y1*j + z1*k + w1)
    // =
    // i*(+x0*w1 + y0*z1 - z0*y1 + w0*x1) +
    // j*(-x0*z1 + y0*w1 + z0*x1 + w0*y1) +
    // k*(+x0*y1 - y0*x1 + z0*w1 + w0*z1) +
    // 1*(-x0*x1 - y0*y1 - z0*z1 + w0*w1)

    return Quaternion<Real>
        (
            +q0[0] * q1[3] + q0[1] * q1[2] - q0[2] * q1[1] + q0[3] * q1[0],
            -q0[0] * q1[2] + q0[1] * q1[3] + q0[2] * q1[0] + q0[3] * q1[1],
            +q0[0] * q1[1] - q0[1] * q1[0] + q0[2] * q1[3] + q0[3] * q1[2],
            -q0[0] * q1[0] - q0[1] * q1[1] - q0[2] * q1[2] + q0[3] * q1[3]
            );
}

template <typename Real>
Quaternion<Real> Inverse(Quaternion<Real> const& q)
{
    Real sqrLen = Dot(q, q);
    if (sqrLen > (Real)0)
    {
        Real invSqrLen = ((Real)1) / sqrLen;
        Quaternion<Real> inverse = Conjugate(q)*invSqrLen;
        return inverse;
    }
    else
    {
        return Quaternion<Real>::Zero();
    }
}

template <typename Real>
Quaternion<Real> Conjugate(Quaternion<Real> const& q)
{
    return Quaternion<Real>(-q[0], -q[1], -q[2], +q[3]);
}

template <typename Real>
Vector<4, Real> Rotate(Quaternion<Real> const& q, Vector<4, Real> const& v)
{
    Vector<4, Real> u = q*Quaternion<Real>(v)*Conjugate(q);

    // Zero-out the w-component in remove numerical round-off error.
    u[3] = (Real)0;
    return u;
}

template <typename Real>
Quaternion<Real> Slerp(Real t, Quaternion<Real> const& q0,
    Quaternion<Real> const& q1)
{
    Real cosA = Dot(q0, q1);
    Real sign;
    if (cosA >= (Real)0)
    {
        sign = (Real)1;
    }
    else
    {
        cosA = -cosA;
        sign = (Real)-1;
    }

    Real f0, f1;
    ChebyshevRatio<Real>::Get(t, cosA, f0, f1);
    return q0 * f0 + q1 * (sign * f1);
}

template <typename Real>
Quaternion<Real> SlerpR(Real t, Quaternion<Real> const& q0,
    Quaternion<Real> const& q1)
{
    Real f0, f1;
    ChebyshevRatio<Real>::Get(t, Dot(q0, q1), f0, f1);
    return q0 * f0 + q1 * f1;
}

template <typename Real>
Quaternion<Real> SlerpRP(Real t, Quaternion<Real> const& q0,
    Quaternion<Real> const& q1, Real cosA)
{
    Real f0, f1;
    ChebyshevRatio<Real>::Get(t, cosA, f0, f1);
    return q0 * f0 + q1 * f1;
}

#endif

}
