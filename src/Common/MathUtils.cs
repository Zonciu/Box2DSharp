using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Box2DSharp.Common
{
    public static class MathUtils
    {
        /// Perform the dot product on two vectors.
        /// 点积,a·b=|a||b|·cosθ
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Dot(in Vector2 a, in Vector2 b)
        {
            return a.X * b.X + a.Y * b.Y;
        }

        /// Perform the cross product on two vectors. In 2D this produces a scalar.
        /// 叉积,axb=|a||b|·sinθ 
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Cross(in Vector2 a, in Vector2 b)
        {
            return a.X * b.Y - a.Y * b.X;
        }

        /// Perform the cross product on a vector and a scalar. In 2D this produces
        /// a vector.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 Cross(in Vector2 a, float s)
        {
            return new Vector2(s * a.Y, -s * a.X);
        }

        /// Perform the cross product on a scalar and a vector. In 2D this produces
        /// a vector.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 Cross(float s, in Vector2 a)
        {
            return new Vector2(-s * a.Y, s * a.X);
        }

        /// Multiply a matrix times a vector. If a rotation matrix is provided,
        /// then this transforms the vector from one frame to another.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 Mul(in Matrix2x2 m, in Vector2 v)
        {
            return new Vector2(m.ex.X * v.X + m.ey.X * v.Y, m.ex.Y * v.X + m.ey.Y * v.Y);
        }

        /// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
        /// then this transforms the vector from one frame to another (inverse transform).
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 MulT(in Matrix2x2 m, in Vector2 v)
        {
            return new Vector2(Dot(v, m.ex), Dot(v, m.ey));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Distance(in Vector2 a, in Vector2 b)
        {
            var c = a - b;
            return c.Length();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float DistanceSquared(in Vector2 a, in Vector2 b)
        {
            var c = a - b;
            return Dot(c, c);
        }

        /// Perform the dot product on two vectors.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Dot(in Vector3 a, in Vector3 b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        /// Perform the cross product on two vectors.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Cross(in Vector3 a, in Vector3 b)
        {
            return new Vector3(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
        }

        // A * B
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix2x2 Mul(in Matrix2x2 a, in Matrix2x2 b)
        {
            return new Matrix2x2(Mul(a, b.ex), Mul(a, b.ey));
        }

        // A^T * B
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix2x2 MulT(in Matrix2x2 a, in Matrix2x2 b)
        {
            var c1 = new Vector2(Dot(a.ex, b.ex), Dot(a.ey, b.ex));

            var c2 = new Vector2(Dot(a.ex, b.ey), Dot(a.ey, b.ey));
            return new Matrix2x2(c1, c2);
        }

        /// Multiply a matrix times a vector.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Mul(in Matrix3x3 m, in Vector3 v)
        {
            return v.X * m.ex + v.Y * m.ey + v.Z * m.ez;
        }

        /// Multiply a matrix times a vector.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 Mul22(in Matrix3x3 m, in Vector2 v)
        {
            return new Vector2(m.ex.X * v.X + m.ey.X * v.Y, m.ex.Y * v.X + m.ey.Y * v.Y);
        }

        /// Multiply two rotations: q * r
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Rotation Mul(in Rotation q, in Rotation r)
        {
            // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
            // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
            // s = qs * rc + qc * rs
            // c = qc * rc - qs * rs
            return new Rotation
            {
                Sin = q.Sin * r.Cos + q.Cos * r.Sin,
                Cos = q.Cos * r.Cos - q.Sin * r.Sin
            };
        }

        /// Transpose multiply two rotations: qT * r
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Rotation MulT(in Rotation q, in Rotation r)
        {
            // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
            // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
            // s = qc * rs - qs * rc
            // c = qc * rc + qs * rs
            Rotation qr;
            qr.Sin = q.Cos * r.Sin - q.Sin * r.Cos;
            qr.Cos = q.Cos * r.Cos + q.Sin * r.Sin;
            return qr;
        }

        /// Rotate a vector
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 Mul(in Rotation q, in Vector2 v)
        {
            return new Vector2(q.Cos * v.X - q.Sin * v.Y, q.Sin * v.X + q.Cos * v.Y);
        }

        /// Inverse rotate a vector
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 MulT(in Rotation q, in Vector2 v)
        {
            return new Vector2(q.Cos * v.X + q.Sin * v.Y, -q.Sin * v.X + q.Cos * v.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 Mul(in Transform T, in Vector2 v)
        {
            float x = (T.Rotation.Cos * v.X - T.Rotation.Sin * v.Y) + T.Position.X;
            float y = (T.Rotation.Sin * v.X + T.Rotation.Cos * v.Y) + T.Position.Y;
            return new Vector2(x, y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 MulT(in Transform T, in Vector2 v)
        {
            float px = v.X - T.Position.X;
            float py = v.Y - T.Position.Y;
            float x  = (T.Rotation.Cos * px + T.Rotation.Sin * py);
            float y  = (-T.Rotation.Sin * px + T.Rotation.Cos * py);
            return new Vector2(x, y);
        }

        // v2 = A.Rotation.Rot(B.Rotation.Rot(v1) + B.p) + A.p
        //    = (A.q * B.q).Rot(v1) + A.Rotation.Rot(B.p) + A.p
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Transform Mul(in Transform A, in Transform B)
        {
            Transform C;
            C.Rotation = Mul(A.Rotation, B.Rotation);
            C.Position = Mul(A.Rotation, B.Position) + A.Position;
            return C;
        }

        // v2 = A.q' * (B.q * v1 + B.p - A.p)
        //    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Transform MulT(in Transform A, in Transform B)
        {
            Transform C;
            C.Rotation = MulT(A.Rotation, B.Rotation);
            C.Position = MulT(A.Rotation, B.Position - A.Position);
            return C;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Clamp(float a, float low, float high)
        {
            return Math.Max(low, Math.Min(a, high));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 Clamp(in Vector2 a, in Vector2 low, in Vector2 high)
        {
            return Vector2.Max(low, Vector2.Min(a, high));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Swap<T>(ref T a, ref T b)
        {
            var tmp = a;
            a = b;
            b = tmp;
        }

        /// "Next Largest Power of 2
        /// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
        /// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
        /// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
        /// largest power of 2. For a 32-bit value:"
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint NextPowerOfTwo(uint x)
        {
            x |= (x >> 1);
            x |= (x >> 2);
            x |= (x >> 4);
            x |= (x >> 8);
            x |= (x >> 16);
            return x + 1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsPowerOfTwo(uint x)
        {
            bool result = x > 0 && (x & (x - 1)) == 0;
            return result;
        }
    }
}