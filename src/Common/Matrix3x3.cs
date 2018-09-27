using System;
using System.Numerics;

namespace Box2DSharp.Common
{
    public struct Matrix3x3
    {
        /// Construct this matrix using columns.
        public Matrix3x3(in Vector3 c1, in Vector3 c2, in Vector3 c3)
        {
            ex = c1;
            ey = c2;
            ez = c3;
        }

        /// Set this matrix to all zeros.
        public void SetZero()
        {
            ex.SetZero();
            ey.SetZero();
            ez.SetZero();
        }

        /// Solve A * x = b, where b is a column vector. This is more efficient
        /// than computing the inverse in one-shot cases.
        public Vector3 Solve33(in Vector3 b)
        {
            var det = MathUtils.Dot(ex, MathUtils.Cross(ey, ez));
            if (!det.Equals(0.0f))
            {
                det = 1.0f / det;
            }

            Vector3 x;
            x.X = det * MathUtils.Dot(b, MathUtils.Cross(ey, ez));
            x.Y = det * MathUtils.Dot(ex, MathUtils.Cross(b, ez));
            x.Z = det * MathUtils.Dot(ex, MathUtils.Cross(ey, b));
            return x;
        }

        /// Solve A * x = b, where b is a column vector. This is more efficient
        /// than computing the inverse in one-shot cases. Solve only the upper
        /// 2-by-2 matrix equation.
        public Vector2 Solve22(in Vector2 b)
        {
            var a11 = ex.X;
            var a12 = ey.X;
            var a21 = ex.Y;
            var a22 = ey.Y;

            var det = a11 * a22 - a12 * a21;
            if (!det.Equals(0.0f))
            {
                det = 1.0f / det;
            }

            Vector2 x;
            x.X = det * (a22 * b.X - a12 * b.Y);
            x.Y = det * (a11 * b.Y - a21 * b.X);
            return x;
        }

        /// Get the inverse of this matrix as a 2-by-2.
        /// Returns the zero matrix if singular.
        public void GetInverse22(ref Matrix3x3 matrix3x3)
        {
            float a   = ex.X, b = ey.X, c = ex.Y, d = ey.Y;
            float det = a * d - b * c;
            if (!det.Equals(0.0f))
            {
                det = 1.0f / det;
            }

            matrix3x3.ex.X = det * d;
            matrix3x3.ey.X = -det * b;
            matrix3x3.ex.Z = 0.0f;
            matrix3x3.ex.Y = -det * c;
            matrix3x3.ey.Y = det * a;
            matrix3x3.ey.Z = 0.0f;
            matrix3x3.ez.X = 0.0f;
            matrix3x3.ez.Y = 0.0f;
            matrix3x3.ez.Z = 0.0f;
        }

        /// Get the symmetric inverse of this matrix as a 3-by-3.
        /// Returns the zero matrix if singular.
        public void GetSymInverse33(ref Matrix3x3 matrix3x3)
        {
            float det = MathUtils.Dot(ex, MathUtils.Cross(ey, ez));
            if (!det.Equals(0.0f))
            {
                det = 1.0f / det;
            }

            float a11 = ex.X, a12 = ey.X, a13 = ez.X;
            float a22 = ey.Y, a23 = ez.Y;
            float a33 = ez.Z;

            matrix3x3.ex.X = det * (a22 * a33 - a23 * a23);
            matrix3x3.ex.Y = det * (a13 * a23 - a12 * a33);
            matrix3x3.ex.Z = det * (a12 * a23 - a13 * a22);

            matrix3x3.ey.X = matrix3x3.ex.Y;
            matrix3x3.ey.Y = det * (a11 * a33 - a13 * a13);
            matrix3x3.ey.Z = det * (a13 * a12 - a11 * a23);

            matrix3x3.ez.X = matrix3x3.ex.Z;
            matrix3x3.ez.Y = matrix3x3.ey.Z;
            matrix3x3.ez.Z = det * (a11 * a22 - a12 * a12);
        }

        public Vector3 ex;

        public Vector3 ey;

        public Vector3 ez;
    }
}