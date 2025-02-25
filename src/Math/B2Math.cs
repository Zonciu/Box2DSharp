using System;
using System.Runtime.CompilerServices;

namespace Box2DSharp
{
    public static class B2Math
    {
        public const float Pi = 3.14159265359f;

        public const float FloatEpsilon = 1.192092896e-07F;

        /// <summary>
        /// Vector dot product
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Dot(Vec2 a, Vec2 b)
        {
            return a.X * b.X + a.Y * b.Y;
        }

        /// <summary>
        /// Vector cross product. In 2D this yields a scalar.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Cross(Vec2 a, Vec2 b)
        {
            return a.X * b.Y - a.Y * b.X;
        }

        /// <summary>
        /// Perform the cross product on a vector and a scalar. In 2D this produces a vector.
        /// </summary>
        /// <param name="v"></param>
        /// <param name="s"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 CrossVS(Vec2 v, float s)
        {
            return new Vec2(s * v.Y, -s * v.X);
        }

        /// <summary>
        /// Perform the cross product on a scalar and a vector. In 2D this produces a vector.
        /// </summary>
        /// <param name="s"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 CrossSV(float s, Vec2 v)
        {
            return new Vec2(-s * v.Y, s * v.X);
        }

        /// <summary>
        /// Get a left pointing perpendicular vector. Equivalent to b2CrossSV(1.0f, v)
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 LeftPerp(Vec2 v)
        {
            return new Vec2(-v.Y, v.X);
        }

        /// <summary>
        /// Get a right pointing perpendicular vector. Equivalent to b2CrossVS(v, 1.0f)
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 RightPerp(Vec2 v)
        {
            return new Vec2(v.Y, -v.X);
        }

        /// <summary>
        /// Vector addition
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 Add(Vec2 a, Vec2 b)
        {
            return new Vec2(a.X + b.X, a.Y + b.Y);
        }

        /// <summary>
        /// Vector subtraction
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 Sub(Vec2 a, Vec2 b)
        {
            return new Vec2(a.X - b.X, a.Y - b.Y);
        }

        /// <summary>
        /// Vector negation
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 Neg(Vec2 a)
        {
            return new Vec2(-a.X, -a.Y);
        }

        /// <summary>
        /// Vector linear interpolation
        /// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="t"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 Lerp(Vec2 a, Vec2 b, float t)
        {
            return new Vec2((1.0f - t) * a.X + t * b.X, (1.0f - t) * a.Y + t * b.Y);
        }

        /// <summary>
        /// Component-wise multiplication
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 Mul(Vec2 a, Vec2 b)
        {
            return new Vec2(a.X * b.X, a.Y * b.Y);
        }

        /// <summary>
        /// Multiply a scalar and vector
        /// </summary>
        /// <param name="s"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 MulSV(float s, Vec2 v)
        {
            return new Vec2(s * v.X, s * v.Y);
        }

        /// <summary>
        /// a + s * b
        /// </summary>
        /// <param name="a"></param>
        /// <param name="s"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 MulAdd(Vec2 a, float s, Vec2 b)
        {
            return new Vec2(a.X + s * b.X, a.Y + s * b.Y);
        }

        /// <summary>
        /// a - s * b
        /// </summary>
        /// <param name="a"></param>
        /// <param name="s"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 MulSub(Vec2 a, float s, Vec2 b)
        {
            return new Vec2(a.X - s * b.X, a.Y - s * b.Y);
        }

        /// <summary>
        /// Component-wise absolute vector
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 Abs(Vec2 a)
        {
            return new(Math.Abs(a.X), Math.Abs(a.Y));
        }

        /// <summary>
        /// Component-wise minimum vector
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 Min(Vec2 a, Vec2 b)
        {
            Vec2 c;
            c.X = Math.Min(a.X, b.X);
            c.Y = Math.Min(a.Y, b.Y);
            return c;
        }

        /// <summary>
        /// Component-wise maximum vector
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 Max(Vec2 a, Vec2 b)
        {
            return new(Math.Max(a.X, b.X), Math.Max(a.Y, b.Y));
        }

        /// <summary>
        /// Component-wise clamp vector v into the range [a, b]
        /// </summary>
        /// <param name="v"></param>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 Clamp(Vec2 v, Vec2 a, Vec2 b)
        {
            Vec2 c;
            c.X = Math.Clamp(v.X, a.X, b.X);
            c.Y = Math.Clamp(v.Y, a.Y, b.Y);
            return c;
        }

        /// <summary>
        /// Get the distance between two points
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Distance(Vec2 a, Vec2 b)
        {
            float dx = b.X - a.X;
            float dy = b.Y - a.Y;
            return MathF.Sqrt(dx * dx + dy * dy);
        }

        /// <summary>
        /// Normalize rotation
        /// </summary>
        /// <param name="q"></param>
        /// <returns></returns>
        public static Rot NormalizeRot(Rot q)
        {
            float mag = MathF.Sqrt(q.S * q.S + q.C * q.C);
            float invMag = mag > 0.0 ? 1.0f / mag : 0.0f;
            Rot qn = new(q.C * invMag, q.S * invMag);
            return qn;
        }

        /// <summary>
        /// Integration rotation from angular velocity
        /// </summary>
        /// <param name="q1">initial rotation</param>
        /// <param name="deltaAngle">the angular displacement in radians</param>
        /// <returns></returns>
        public static Rot IntegrateRotation(Rot q1, float deltaAngle)
        {
            // dc/dt = -omega * sin(t)
            // ds/dt = omega * cos(t)
            // c2 = c1 - omega * h * s1
            // s2 = s1 + omega * h * c1
            Rot q2 = new(q1.C - deltaAngle * q1.S, q1.S + deltaAngle * q1.C);
            float mag = MathF.Sqrt(q2.S * q2.S + q2.C * q2.C);
            float invMag = mag > 0.0 ? 1.0f / mag : 0.0f;
            Rot qn = new(q2.C * invMag, q2.S * invMag);
            return qn;
        }

        /// <summary>
        /// Get the length squared of this vector
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float LengthSquared(Vec2 v)
        {
            return v.X * v.X + v.Y * v.Y;
        }

        /// <summary>
        /// Get the distance squared between points
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float DistanceSquared(Vec2 a, Vec2 b)
        {
            Vec2 c = new(b.X - a.X, b.Y - a.Y);
            return c.X * c.X + c.Y * c.Y;
        }

        /// <summary>
        /// Normalized linear interpolation
        /// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
        /// </summary>
        /// <param name="q1"></param>
        /// <param name="q2"></param>
        /// <param name="t"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Rot NLerp(Rot q1, Rot q2, float t)
        {
            float omt = 1.0f - t;
            Rot q = new(
                omt * q1.C + t * q2.C,
                omt * q1.S + t * q2.S);

            return NormalizeRot(q);
        }

        /// <summary>
        /// Compute the angular velocity necessary to rotate between two rotations over a give time
        /// </summary>
        /// <param name="q1">initial rotation</param>
        /// <param name="q2">final rotation</param>
        /// <param name="inv_h">inverse time step</param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float ComputeAngularVelocity(Rot q1, Rot q2, float inv_h)
        {
            // ds/dt = omega * cos(t)
            // dc/dt = -omega * sin(t)
            // s2 = s1 + omega * h * c1
            // c2 = c1 - omega * h * s1

            // omega * h * s1 = c1 - c2
            // omega * h * c1 = s2 - s1
            // omega * h = (c1 - c2) * s1 + (s2 - s1) * c1;
            // omega * h = s1 * c1 - c2 * s1 + s2 * c1 - s1 * c1
            // omega * h = s2 * c1 - c2 * s1 = sin(a2 - a1) ~= a2 - a1 for small delta
            float omega = inv_h * (q2.S * q1.C - q2.C * q1.S);
            return omega;
        }

        /// <summary>
        /// Multiply two rotations: q * r
        /// </summary>
        /// <param name="q"></param>
        /// <param name="r"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Rot MulRot(Rot q, Rot r)
        {
            // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
            // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
            // s(q + r) = qs * rc + qc * rs
            // c(q + r) = qc * rc - qs * rs
            Rot qr;
            qr.S = q.S * r.C + q.C * r.S;
            qr.C = q.C * r.C - q.S * r.S;
            return qr;
        }

        /// <summary>
        /// Transpose multiply two rotations: qT * r
        /// </summary>
        /// <param name="q"></param>
        /// <param name="r"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Rot InvMulRot(Rot q, Rot r)
        {
            // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
            // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
            // s(q - r) = qc * rs - qs * rc
            // c(q - r) = qc * rc + qs * rs
            Rot qr;
            qr.S = q.C * r.S - q.S * r.C;
            qr.C = q.C * r.C + q.S * r.S;
            return qr;
        }

        /// <summary>
        /// relative angle between b and a (rot_b * inv(rot_a))
        /// </summary>
        /// <param name="b"></param>
        /// <param name="a"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float RelativeAngle(Rot b, Rot a)
        {
            // sin(b - a) = bs * ac - bc * as
            // cos(b - a) = bc * ac + bs * as
            float s = b.S * a.C - b.C * a.S;
            float c = b.C * a.C + b.S * a.S;
            return Atan2(s, c);
        }

        /// <summary>
        /// Convert an angle in the range [-2*pi, 2*pi] into the range [-pi, pi]
        /// </summary>
        /// <param name="angle"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float UnwindAngle(float angle)
        {
            return angle switch
            {
                < -Pi => angle + 2.0f * Pi,
                > Pi => angle - 2.0f * Pi,
                _ => angle
            };
        }

        /// <summary>
        /// Convert any into the range [-pi, pi] (slow)
        /// </summary>
        /// <param name="angle"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float UnwindLargeAngle(float angle)
        {
            while (angle > Pi)
            {
                angle -= 2.0f * Pi;
            }

            while (angle < -Pi)
            {
                angle += 2.0f * Pi;
            }

            return angle;
        }

        /// <summary>
        /// Rotate a vector
        /// </summary>
        /// <param name="q"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 RotateVector(Rot q, Vec2 v)
        {
            return new Vec2(q.C * v.X - q.S * v.Y, q.S * v.X + q.C * v.Y);
        }

        /// <summary>
        /// Inverse rotate a vector
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 InvRotateVector(Rot q, Vec2 v)
        {
            return new Vec2(q.C * v.X + q.S * v.Y, -q.S * v.X + q.C * v.Y);
        }

        /// <summary>
        /// Transform a point (e.g. local space to world space)
        /// </summary>
        /// <param name="t"></param>
        /// <param name="p"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 TransformPoint(Transform t, Vec2 p)
        {
            float x = (t.Q.C * p.X - t.Q.S * p.Y) + t.P.X;
            float y = (t.Q.S * p.X + t.Q.C * p.Y) + t.P.Y;

            return new Vec2(x, y);
        }

        /// <summary>
        /// Inverse transform a point (e.g. world space to local space)
        /// </summary>
        /// <param name="t"></param>
        /// <param name="p"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 InvTransformPoint(Transform t, Vec2 p)
        {
            float vx = p.X - t.P.X;
            float vy = p.Y - t.P.Y;
            return new Vec2(t.Q.C * vx + t.Q.S * vy, -t.Q.S * vx + t.Q.C * vy);
        }

        /// <summary>
        /// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
        ///    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
        /// </summary>
        /// <param name="A"></param>
        /// <param name="B"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Transform MulTransforms(Transform A, Transform B)
        {
            Transform C;
            C.Q = MulRot(A.Q, B.Q);
            C.P = Add(RotateVector(A.Q, B.P), A.P);
            return C;
        }

        /// <summary>
        /// v2 = A.q' * (B.q * v1 + B.p - A.p)
        ///    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
        /// </summary>
        /// <param name="A"></param>
        /// <param name="B"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Transform InvMulTransforms(Transform A, Transform B)
        {
            Transform C;
            C.Q = InvMulRot(A.Q, B.Q);
            C.P = InvRotateVector(A.Q, Sub(B.P, A.P));
            return C;
        }

        /// <summary>
        /// Multiply a 2-by-2 matrix times a 2D vector
        /// </summary>
        /// <param name="A"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 MulMV(Mat22 A, Vec2 v)
        {
            Vec2 u =
                new(
                    A.Cx.X * v.X + A.Cy.X * v.Y,
                    A.Cx.Y * v.X + A.Cy.Y * v.Y);
            return u;
        }

        /// <summary>
        /// Get the inverse of a 2-by-2 matrix
        /// </summary>
        /// <param name="A"></param>
        /// <returns></returns>
        public static Mat22 GetInverse22(Mat22 A)
        {
            float a = A.Cx.X, b = A.Cy.X, c = A.Cx.Y, d = A.Cy.Y;
            float det = a * d - b * c;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }

            Mat22 B = new(
                new(det * d, -det * c),
                new(-det * b, det * a));
            return B;
        }

        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient
        /// than computing the inverse in one-shot cases.
        /// </summary>
        /// <param name="A"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static Vec2 Solve22(Mat22 A, Vec2 b)
        {
            float a11 = A.Cx.X, a12 = A.Cy.X, a21 = A.Cx.Y, a22 = A.Cy.Y;
            float det = a11 * a22 - a12 * a21;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }

            Vec2 x = new(det * (a22 * b.X - a12 * b.Y), det * (a11 * b.Y - a21 * b.X));
            return x;
        }

        /// <summary>
        /// Does a fully contain b
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool AABB_Contains(AABB a, AABB b)
        {
            bool s = true;
            s = s && a.LowerBound.X <= b.LowerBound.X;
            s = s && a.LowerBound.Y <= b.LowerBound.Y;
            s = s && b.UpperBound.X <= a.UpperBound.X;
            s = s && b.UpperBound.Y <= a.UpperBound.Y;
            return s;
        }

        /// <summary>
        /// Get the center of the AABB.
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 AABB_Center(AABB a)
        {
            Vec2 b = new(0.5f * (a.LowerBound.X + a.UpperBound.X), 0.5f * (a.LowerBound.Y + a.UpperBound.Y));
            return b;
        }

        /// <summary>
        /// Get the extents of the AABB (half-widths).
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 AABB_Extents(AABB a)
        {
            Vec2 b = new(0.5f * (a.UpperBound.X - a.LowerBound.X), 0.5f * (a.UpperBound.Y - a.LowerBound.Y));
            return b;
        }

        /// <summary>
        /// Union of two AABBs
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static AABB AABB_Union(AABB a, AABB b)
        {
            AABB c;
            c.LowerBound.X = Math.Min(a.LowerBound.X, b.LowerBound.X);
            c.LowerBound.Y = Math.Min(a.LowerBound.Y, b.LowerBound.Y);
            c.UpperBound.X = Math.Max(a.UpperBound.X, b.UpperBound.X);
            c.UpperBound.Y = Math.Max(a.UpperBound.Y, b.UpperBound.Y);
            return c;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsValid(float a)
        {
            return !float.IsNaN(a) && !float.IsInfinity(a);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Vec2_IsValid(Vec2 v)
        {
            return !float.IsNaN(v.X) && !float.IsNaN(v.Y) && !float.IsInfinity(v.X) && !float.IsInfinity(v.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Rot_IsValid(Rot q)
        {
            return !float.IsNaN(q.S) && !float.IsNaN(q.C) && !float.IsInfinity(q.S) && !float.IsInfinity(q.C) && q.IsNormalized();
        }

        /// <summary>
        /// https://mazzo.li/posts/vectorized-atan2.html
        /// </summary>
        /// <param name="x"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Atan(float x)
        {
            const float a1 = 0.99997726f;
            const float a3 = -0.33262347f;
            const float a5 = 0.19354346f;
            const float a7 = -0.11643287f;
            const float a9 = 0.05265332f;
            const float a11 = -0.01172120f;

            float x2 = x * x;
            return x * (a1 + x2 * (a3 + x2 * (a5 + x2 * (a7 + x2 * (a9 + x2 * a11)))));
        }

        /// <summary>
        /// Compute an approximate arctangent in the range [-pi, pi]
        /// This is hand coded for cross platform determinism. The atan2f
        ///	function in the standard library is not cross platform deterministic.
        /// I tested atan2f and got different results on Apple Clang (Arm) than MSVC (x64).
        /// </summary>
        /// <param name="y"></param>
        /// <param name="x"></param>
        /// <returns></returns>
        public static float Atan2(float y, float x)
        {
            float pi = Pi;
            float halfPi = 0.5f * Pi;

            bool swap = Math.Abs(x) < Math.Abs(y);
            float atanInput = (swap ? x : y) / (swap ? y : x);

            // Approximate atan
            float res = Atan(atanInput);

            // If swapped, adjust atan output
            res = swap ? (atanInput >= 0.0f ? halfPi : -halfPi) - res : res;

            // Adjust quadrants
            if (x >= 0.0f && y >= 0.0f)
            { } // 1st quadrant
            else if (x < 0.0f && y >= 0.0f)
            {
                res = pi + res;
            } // 2nd quadrant
            else if (x < 0.0f && y < 0.0f)
            {
                res = -pi + res;
            } // 3rd quadrant
            else if (x >= 0.0f && y < 0.0f)
            { } // 4th quadrant

            return res;
        }

        /// <summary>
        /// Make a rotation using an angle in radians
        /// Approximate cosine and sine for determinism. In my testing cosf and sinf produced
        /// the same results on x64 and ARM using MSVC, GCC, and Clang. However, I don't trust
        /// this result.
        /// https://en.wikipedia.org/wiki/Bh%C4%81skara_I%27s_sine_approximation_formula
        /// </summary>
        /// <param name="angle"></param>
        /// <returns></returns>
        public static Rot MakeRot(float angle)
        {
            // return ( b2Rot ){ cosf( angle ), sinf( angle ) };

            var x = UnwindLargeAngle(angle);
            var pi2 = Pi * Pi;

            Rot q;

            // cosine needs angle in [-pi/2, pi/2]
            if (x < -0.5f * Pi)
            {
                var y = x + Pi;
                var y2 = y * y;
                q.C = -(pi2 - 4.0f * y2) / (pi2 + y2);
            }
            else if (x > 0.5f * Pi)
            {
                var y = x - Pi;
                var y2 = y * y;
                q.C = -(pi2 - 4.0f * y2) / (pi2 + y2);
            }
            else
            {
                var y2 = x * x;
                q.C = (pi2 - 4.0f * y2) / (pi2 + y2);
            }

            // sine needs angle in [0, pi]
            if (x < 0.0f)
            {
                var y = x + Pi;
                q.S = -16.0f * y * (Pi - y) / (5.0f * pi2 - 4.0f * y * (Pi - y));
            }
            else
            {
                q.S = 16.0f * x * (Pi - x) / (5.0f * pi2 - 4.0f * x * (Pi - x));
            }

            q = NormalizeRot(q);
            return q;
        }
    }
}