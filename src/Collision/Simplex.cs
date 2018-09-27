using System;
using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Collision
{
    public struct Simplex
    {
        public SimplexVertex[] Vertices; //20

        //m_v1, m_v2, m_v3;

        public int Count;

        public void ReadCache(
            ref SimplexCache  cache,
            in  DistanceProxy proxyA,
            in  Transform     transformA,
            in  DistanceProxy proxyB,
            in  Transform     transformB)
        {
            Debug.Assert(cache.Count <= 3);

            // Copy data from cache.
            Count    = cache.Count;
            Vertices = new SimplexVertex[3];

            //ref b2SimplexVertex vertices = ref m_v1;
            for (var i = 0; i < Count; ++i)
            {
                ref var v = ref Vertices[i];
                v.indexA = cache.IndexA[i];
                v.indexB = cache.IndexB[i];
                var wALocal = proxyA.GetVertex(v.indexA);
                var wBLocal = proxyB.GetVertex(v.indexB);
                v.wA = MathUtils.Mul(transformA, wALocal);
                v.wB = MathUtils.Mul(transformB, wBLocal);
                v.w  = v.wB - v.wA;
                v.a  = 0.0f;
            }

            // Compute the new simplex metric, if it is substantially different than
            // old metric then flush the simplex.
            if (Count > 1)
            {
                var metric1 = cache.Metric;
                var metric2 = GetMetric();
                if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < Settings.Epsilon)
                {
                    // Reset the simplex.
                    Count = 0;
                }
            }

            // If the cache is empty or invalid ...
            if (Count == 0)
            {
                ref var v = ref Vertices[0];
                v.indexA = 0;
                v.indexB = 0;
                var wALocal = proxyA.GetVertex(0);
                var wBLocal = proxyB.GetVertex(0);
                v.wA  = MathUtils.Mul(transformA, wALocal);
                v.wB  = MathUtils.Mul(transformB, wBLocal);
                v.w   = v.wB - v.wA;
                v.a   = 1.0f;
                Count = 1;
            }
        }

        public void WriteCache(ref SimplexCache cache)
        {
            cache.Metric = GetMetric();
            cache.Count  = (ushort) Count;
            for (var i = 0; i < Count; ++i)
            {
                cache.IndexA[i] = (byte) Vertices[i].indexA;
                cache.IndexB[i] = (byte) Vertices[i].indexB;
            }
        }

        public Vector2 GetSearchDirection()
        {
            switch (Count)
            {
            case 1:
                return -Vertices[0].w;

            case 2:
            {
                var e12 = Vertices[1].w - Vertices[0].w;
                var sgn = MathUtils.Cross(e12, -Vertices[0].w);
                if (sgn > 0.0f)
                {
                    // Origin is left of e12.
                    return MathUtils.Cross(1.0f, e12);
                }

                // Origin is right of e12.
                return MathUtils.Cross(e12, 1.0f);
            }

            default:
                throw new ArgumentOutOfRangeException(nameof(Count));
            }
        }

        public Vector2 GetClosestPoint()
        {
            switch (Count)
            {
            case 1:
                return Vertices[0].w;

            case 2:
                return Vertices[0].a * Vertices[0].w + Vertices[1].a * Vertices[1].w;

            case 3:
                return Vector2.Zero;

            default:
                throw new ArgumentOutOfRangeException(nameof(Count));
            }
        }

        public void GetWitnessPoints(out Vector2 pA, out Vector2 pB)
        {
            switch (Count)
            {
            case 1:
                pA = Vertices[0].wA;
                pB = Vertices[0].wB;
                break;

            case 2:
                pA = Vertices[0].a * Vertices[0].wA + Vertices[1].a * Vertices[1].wA;
                pB = Vertices[0].a * Vertices[0].wB + Vertices[1].a * Vertices[1].wB;
                break;

            case 3:
                pA = Vertices[0].a * Vertices[0].wA + Vertices[1].a * Vertices[1].wA + Vertices[2].a * Vertices[2].wA;
                pB = pA;
                break;

            default:
                throw new ArgumentOutOfRangeException(nameof(Count));
            }
        }

        public float GetMetric()
        {
            switch (Count)
            {
            case 0:
                Debug.Assert(false);
                return 0.0f;

            case 1:
                return 0.0f;

            case 2:
                return MathUtils.Distance(Vertices[0].w, Vertices[1].w);

            case 3:
                return MathUtils.Cross(Vertices[1].w - Vertices[0].w, Vertices[2].w - Vertices[0].w);

            default:
                throw new ArgumentOutOfRangeException(nameof(Count));
            }
        }

        // Solve a line segment using barycentric coordinates.
        //
        // p = a1 * w1 + a2 * w2
        // a1 + a2 = 1
        //
        // The vector from the origin to the closest point on the line is
        // perpendicular to the line.
        // e12 = w2 - w1
        // dot(p, e) = 0
        // a1 * dot(w1, e) + a2 * dot(w2, e) = 0
        //
        // 2-by-2 linear system
        // [1      1     ][a1] = [1]
        // [w1.e12 w2.e12][a2] = [0]
        //
        // Define
        // d12_1 =  dot(w2, e12)
        // d12_2 = -dot(w1, e12)
        // d12 = d12_1 + d12_2
        //
        // Solution
        // a1 = d12_1 / d12
        // a2 = d12_2 / d12
        public void Solve2()
        {
            var w1  = Vertices[0].w;
            var w2  = Vertices[1].w;
            var e12 = w2 - w1;

            // w1 region
            var d12_2 = -MathUtils.Dot(w1, e12);
            if (d12_2 <= 0.0f)
            {
                // a2 <= 0, so we clamp it to 0
                Vertices[0].a = 1.0f;
                Count         = 1;
                return;
            }

            // w2 region
            var d12_1 = MathUtils.Dot(w2, e12);
            if (d12_1 <= 0.0f)
            {
                // a1 <= 0, so we clamp it to 0
                Vertices[1].a = 1.0f;
                Count         = 1;
                Vertices[0]   = Vertices[1];
                return;
            }

            // Must be in e12 region.
            var inv_d12 = 1.0f / (d12_1 + d12_2);
            Vertices[0].a = d12_1 * inv_d12;
            Vertices[1].a = d12_2 * inv_d12;
            Count         = 2;
        }

        // Possible regions:
        // - points[2]
        // - edge points[0]-points[2]
        // - edge points[1]-points[2]
        // - inside the triangle
        public void Solve3()
        {
            var w1 = Vertices[0].w;
            var w2 = Vertices[1].w;
            var w3 = Vertices[2].w;

            // Edge12
            // [1      1     ][a1] = [1]
            // [w1.e12 w2.e12][a2] = [0]
            // a3 = 0
            var e12   = w2 - w1;
            var w1e12 = MathUtils.Dot(w1, e12);
            var w2e12 = MathUtils.Dot(w2, e12);
            var d12_1 = w2e12;
            var d12_2 = -w1e12;

            // Edge13
            // [1      1     ][a1] = [1]
            // [w1.e13 w3.e13][a3] = [0]
            // a2 = 0
            var e13   = w3 - w1;
            var w1e13 = MathUtils.Dot(w1, e13);
            var w3e13 = MathUtils.Dot(w3, e13);
            var d13_1 = w3e13;
            var d13_2 = -w1e13;

            // Edge23
            // [1      1     ][a2] = [1]
            // [w2.e23 w3.e23][a3] = [0]
            // a1 = 0
            var e23   = w3 - w2;
            var w2e23 = MathUtils.Dot(w2, e23);
            var w3e23 = MathUtils.Dot(w3, e23);
            var d23_1 = w3e23;
            var d23_2 = -w2e23;

            // Triangle123
            var n123 = MathUtils.Cross(e12, e13);

            var d123_1 = n123 * MathUtils.Cross(w2, w3);
            var d123_2 = n123 * MathUtils.Cross(w3, w1);
            var d123_3 = n123 * MathUtils.Cross(w1, w2);

            // w1 region
            if (d12_2 <= 0.0f && d13_2 <= 0.0f)
            {
                Vertices[0].a = 1.0f;
                Count         = 1;
                return;
            }

            // e12
            if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
            {
                var inv_d12 = 1.0f / (d12_1 + d12_2);
                Vertices[0].a = d12_1 * inv_d12;
                Vertices[1].a = d12_2 * inv_d12;
                Count         = 2;
                return;
            }

            // e13
            if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
            {
                var inv_d13 = 1.0f / (d13_1 + d13_2);
                Vertices[0].a = d13_1 * inv_d13;
                Vertices[2].a = d13_2 * inv_d13;
                Count         = 2;
                Vertices[1]   = Vertices[2];
                return;
            }

            // w2 region
            if (d12_1 <= 0.0f && d23_2 <= 0.0f)
            {
                Vertices[1].a = 1.0f;
                Count         = 1;
                Vertices[0]   = Vertices[1];
                return;
            }

            // w3 region
            if (d13_1 <= 0.0f && d23_1 <= 0.0f)
            {
                Vertices[2].a = 1.0f;
                Count         = 1;
                Vertices[0]   = Vertices[2];
                return;
            }

            // e23
            if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
            {
                var inv_d23 = 1.0f / (d23_1 + d23_2);
                Vertices[1].a = d23_1 * inv_d23;
                Vertices[2].a = d23_2 * inv_d23;
                Count         = 2;
                Vertices[0]   = Vertices[2];
                return;
            }

            // Must be in triangle123
            var inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
            Vertices[0].a = d123_1 * inv_d123;
            Vertices[1].a = d123_2 * inv_d123;
            Vertices[2].a = d123_3 * inv_d123;
            Count         = 3;
        }
    }

    /// Used to warm start b2Distance.
    /// Set count to zero on first call.
    public struct SimplexCache
    {
        /// length or area
        public float Metric;

        public ushort Count;

        /// vertices on shape A
        public byte[] IndexA;

        /// vertices on shape B
        public byte[] IndexB;

        public static SimplexCache Create()
        {
            return new SimplexCache
            {
                Metric = 0,
                Count  = 0,
                IndexA = new byte[3],
                IndexB = new byte[3]
            };
        }
    }
}