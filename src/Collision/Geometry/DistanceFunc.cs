using System;
using System.Diagnostics;

namespace Box2DSharp
{
    public static class DistanceFunc
    {
        public static Transform GetSweepTransform(in Sweep sweep, float time)
        {
            // https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
            Transform xf;
            xf.P = B2Math.Add(B2Math.MulSV(1.0f - time, sweep.C1), B2Math.MulSV(time, sweep.C2));

            Rot q = new(
                (1.0f - time) * sweep.Q1.C + time * sweep.Q2.C,
                (1.0f - time) * sweep.Q1.S + time * sweep.Q2.S);

            xf.Q = B2Math.NormalizeRot(q);

            // Shift to origin
            xf.P = B2Math.Sub(xf.P, B2Math.RotateVector(xf.Q, sweep.LocalCenter));
            return xf;
        }

        /// Follows Ericson 5.1.9 Closest Points of Two Line Segments
        public static SegmentDistanceResult SegmentDistance(Vec2 p1, Vec2 q1, Vec2 p2, Vec2 q2)
        {
            SegmentDistanceResult result = new();

            Vec2 d1 = B2Math.Sub(q1, p1);
            Vec2 d2 = B2Math.Sub(q2, p2);
            Vec2 r = B2Math.Sub(p1, p2);
            float dd1 = B2Math.Dot(d1, d1);
            float dd2 = B2Math.Dot(d2, d2);
            float rd2 = B2Math.Dot(r, d2);
            float rd1 = B2Math.Dot(r, d1);

            const float epsSqr = float.Epsilon * float.Epsilon;

            if (dd1 < epsSqr || dd2 < epsSqr)
            {
                // Handle all degeneracies
                if (dd1 >= epsSqr)
                {
                    // Segment 2 is degenerate
                    result.Fraction1 = Math.Clamp(-rd1 / dd1, 0.0f, 1.0f);
                    result.Fraction2 = 0.0f;
                }
                else if (dd2 >= epsSqr)
                {
                    // Segment 1 is degenerate
                    result.Fraction1 = 0.0f;
                    result.Fraction2 = Math.Clamp(rd2 / dd2, 0.0f, 1.0f);
                }
                else
                {
                    result.Fraction1 = 0.0f;
                    result.Fraction2 = 0.0f;
                }
            }
            else
            {
                // Non-degenerate segments
                float d12 = B2Math.Dot(d1, d2);

                float denom = dd1 * dd2 - d12 * d12;

                // Fraction on segment 1
                float f1 = 0.0f;
                if (denom != 0.0f)
                {
                    // not parallel
                    f1 = Math.Clamp((d12 * rd2 - rd1 * dd2) / denom, 0.0f, 1.0f);
                }

                // Compute point on segment 2 closest to p1 + f1 * d1
                float f2 = (d12 * f1 + rd2) / dd2;

                // Clamping of segment 2 requires a do over on segment 1
                if (f2 < 0.0f)
                {
                    f2 = 0.0f;
                    f1 = Math.Clamp(-rd1 / dd1, 0.0f, 1.0f);
                }
                else if (f2 > 1.0f)
                {
                    f2 = 1.0f;
                    f1 = Math.Clamp((d12 - rd1) / dd1, 0.0f, 1.0f);
                }

                result.Fraction1 = f1;
                result.Fraction2 = f2;
            }

            result.Closest1 = B2Math.MulAdd(p1, result.Fraction1, d1);
            result.Closest2 = B2Math.MulAdd(p2, result.Fraction2, d2);
            result.DistanceSquared = B2Math.DistanceSquared(result.Closest1, result.Closest2);
            return result;
        }

        public static DistanceProxy MakeProxy(Vec2 vec, float radius)
        {
            DistanceProxy proxy = new();
            proxy.Points[0] = vec;
            proxy.Count = 1;
            proxy.Radius = radius;
            return proxy;
        }

        // GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
        // todo try not copying
        public static DistanceProxy MakeProxy(Span<Vec2> vertices, int count, float radius)
        {
            count = Math.Min(count, Core.MaxPolygonVertices);
            DistanceProxy proxy = new();
            for (int i = 0; i < count; ++i)
            {
                proxy.Points[i] = vertices[i];
            }

            proxy.Count = count;
            proxy.Radius = radius;
            return proxy;
        }

        public static Vec2 Weight2(float a1, Vec2 w1, float a2, Vec2 w2)
        {
            return new Vec2(a1 * w1.X + a2 * w2.X, a1 * w1.Y + a2 * w2.Y);
        }

        public static Vec2 Weight3(float a1, Vec2 w1, float a2, Vec2 w2, float a3, Vec2 w3)
        {
            return new Vec2(a1 * w1.X + a2 * w2.X + a3 * w3.X, a1 * w1.Y + a2 * w2.Y + a3 * w3.Y);
        }

        public static int FindSupport(in DistanceProxy proxy, in Vec2 direction)
        {
            int bestIndex = 0;
            float bestValue = B2Math.Dot(proxy.Points[0], direction);
            for (int i = 1; i < proxy.Count; ++i)
            {
                float value = B2Math.Dot(proxy.Points[i], direction);
                if (value > bestValue)
                {
                    bestIndex = i;
                    bestValue = value;
                }
            }

            return bestIndex;
        }

        private static Simplex MakeSimplexFromCache(
            in DistanceCache cache,
            in DistanceProxy proxyA,
            in Transform transformA,
            in DistanceProxy proxyB,
            in Transform transformB)
        {
            Debug.Assert(cache.Count <= 3);
            Simplex s = new();

            // Copy data from cache.
            s.Count = cache.Count;

            var vertices = s.Vertices;
            for (int i = 0; i < s.Count; ++i)
            {
                ref SimplexVertex v = ref vertices[i];
                v.IndexA = cache.IndexA[i];
                v.IndexB = cache.IndexB[i];
                Vec2 wALocal = proxyA.Points[v.IndexA];
                Vec2 wBLocal = proxyB.Points[v.IndexB];
                v.WA = B2Math.TransformPoint(transformA, wALocal);
                v.WB = B2Math.TransformPoint(transformB, wBLocal);
                v.W = B2Math.Sub(v.WB, v.WA);

                // invalid
                v.A = -1.0f;
            }

            // If the cache is empty or invalid ...
            if (s.Count == 0)
            {
                ref SimplexVertex v = ref vertices[0];
                v.IndexA = 0;
                v.IndexB = 0;
                Vec2 wALocal = proxyA.Points[0];
                Vec2 wBLocal = proxyB.Points[0];
                v.WA = B2Math.TransformPoint(transformA, wALocal);
                v.WB = B2Math.TransformPoint(transformB, wBLocal);
                v.W = B2Math.Sub(v.WB, v.WA);
                v.A = 1.0f;
                s.Count = 1;
            }

            return s;
        }

        public static void MakeSimplexCache(ref DistanceCache cache, ref Simplex simplex)
        {
            cache.Count = (ushort)simplex.Count;
            var vertices = simplex.Vertices;
            for (int i = 0; i < simplex.Count; ++i)
            {
                cache.IndexA[i] = (byte)vertices[i].IndexA;
                cache.IndexB[i] = (byte)vertices[i].IndexB;
            }
        }

        // Compute the search direction from the current simplex.
        // This is the vector pointing from the closest point on the simplex
        // to the origin.
        // A more accurate search direction can be computed by using the normal
        // vector of the simplex. For example, the normal vector of a line segment
        // can be computed more accurately because it does not involve barycentric
        // coordinates.
        public static Vec2 ComputeSimplexSearchDirection(ref Simplex simplex)
        {
            switch (simplex.Count)
            {
            case 1:
                return B2Math.Neg(simplex.V1.W);

            case 2:
            {
                Vec2 e12 = B2Math.Sub(simplex.V2.W, simplex.V1.W);
                float sgn = B2Math.Cross(e12, B2Math.Neg(simplex.V1.W));
                if (sgn > 0.0f)
                {
                    // Origin is left of e12.
                    return B2Math.LeftPerp(e12);
                }
                else
                {
                    // Origin is right of e12.
                    return B2Math.RightPerp(e12);
                }
            }

            default:
                Debug.Assert(false);
                return Vec2.Zero;
            }
        }

        public static Vec2 ComputeSimplexClosestPoint(ref Simplex s)
        {
            switch (s.Count)
            {
            case 0:
                Debug.Assert(false);
                return Vec2.Zero;

            case 1:
                return s.V1.W;

            case 2:
                return Weight2(s.V1.A, s.V1.W, s.V2.A, s.V2.W);

            case 3:
                return Vec2.Zero;

            default:
                Debug.Assert(false);
                return Vec2.Zero;
            }
        }

        public static void ComputeSimplexWitnessPoints(out Vec2 a, out Vec2 b, ref Simplex s)
        {
            switch (s.Count)
            {
            case 0:
                Debug.Assert(false);

                a = default;
                b = default;
                break;

            case 1:
                a = s.V1.WA;
                b = s.V1.WB;
                break;

            case 2:
                a = Weight2(s.V1.A, s.V1.WA, s.V2.A, s.V2.WA);
                b = Weight2(s.V1.A, s.V1.WB, s.V2.A, s.V2.WB);
                break;

            case 3:
                a = Weight3(s.V1.A, s.V1.WA, s.V2.A, s.V2.WA, s.V3.A, s.V3.WA);

                // TODO_ERIN why are these not equal?
                //*b = b2Weight3(s.v1.a, s.v1.wB, s.v2.a, s.v2.wB, s.v3.a, s.v3.wB);
                b = a;
                break;

            default:
                Debug.Assert(false);

                a = default;
                b = default;
                break;
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
        public static void SolveSimplex2(ref Simplex s)
        {
            Vec2 w1 = s.V1.W;
            Vec2 w2 = s.V2.W;
            Vec2 e12 = B2Math.Sub(w2, w1);

            // w1 region
            float d12_2 = -B2Math.Dot(w1, e12);
            if (d12_2 <= 0.0f)
            {
                // a2 <= 0, so we clamp it to 0
                s.V1.A = 1.0f;
                s.Count = 1;
                return;
            }

            // w2 region
            float d12_1 = B2Math.Dot(w2, e12);
            if (d12_1 <= 0.0f)
            {
                // a1 <= 0, so we clamp it to 0
                s.V2.A = 1.0f;
                s.Count = 1;
                s.V1 = s.V2;
                return;
            }

            // Must be in e12 region.
            float inv_d12 = 1.0f / (d12_1 + d12_2);
            s.V1.A = d12_1 * inv_d12;
            s.V2.A = d12_2 * inv_d12;
            s.Count = 2;
        }

        public static void SolveSimplex3(ref Simplex s)
        {
            Vec2 w1 = s.V1.W;
            Vec2 w2 = s.V2.W;
            Vec2 w3 = s.V3.W;

            // Edge12
            // [1      1     ][a1] = [1]
            // [w1.e12 w2.e12][a2] = [0]
            // a3 = 0
            Vec2 e12 = B2Math.Sub(w2, w1);
            float w1e12 = B2Math.Dot(w1, e12);
            float w2e12 = B2Math.Dot(w2, e12);
            float d12_1 = w2e12;
            float d12_2 = -w1e12;

            // Edge13
            // [1      1     ][a1] = [1]
            // [w1.e13 w3.e13][a3] = [0]
            // a2 = 0
            Vec2 e13 = B2Math.Sub(w3, w1);
            float w1e13 = B2Math.Dot(w1, e13);
            float w3e13 = B2Math.Dot(w3, e13);
            float d13_1 = w3e13;
            float d13_2 = -w1e13;

            // Edge23
            // [1      1     ][a2] = [1]
            // [w2.e23 w3.e23][a3] = [0]
            // a1 = 0
            Vec2 e23 = B2Math.Sub(w3, w2);
            float w2e23 = B2Math.Dot(w2, e23);
            float w3e23 = B2Math.Dot(w3, e23);
            float d23_1 = w3e23;
            float d23_2 = -w2e23;

            // Triangle123
            float n123 = B2Math.Cross(e12, e13);

            float d123_1 = n123 * B2Math.Cross(w2, w3);
            float d123_2 = n123 * B2Math.Cross(w3, w1);
            float d123_3 = n123 * B2Math.Cross(w1, w2);

            // w1 region
            if (d12_2 <= 0.0f && d13_2 <= 0.0f)
            {
                s.V1.A = 1.0f;
                s.Count = 1;
                return;
            }

            // e12
            if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
            {
                float inv_d12 = 1.0f / (d12_1 + d12_2);
                s.V1.A = d12_1 * inv_d12;
                s.V2.A = d12_2 * inv_d12;
                s.Count = 2;
                return;
            }

            // e13
            if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
            {
                float inv_d13 = 1.0f / (d13_1 + d13_2);
                s.V1.A = d13_1 * inv_d13;
                s.V3.A = d13_2 * inv_d13;
                s.Count = 2;
                s.V2 = s.V3;
                return;
            }

            // w2 region
            if (d12_1 <= 0.0f && d23_2 <= 0.0f)
            {
                s.V2.A = 1.0f;
                s.Count = 1;
                s.V1 = s.V2;
                return;
            }

            // w3 region
            if (d13_1 <= 0.0f && d23_1 <= 0.0f)
            {
                s.V3.A = 1.0f;
                s.Count = 1;
                s.V1 = s.V3;
                return;
            }

            // e23
            if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
            {
                float inv_d23 = 1.0f / (d23_1 + d23_2);
                s.V2.A = d23_1 * inv_d23;
                s.V3.A = d23_2 * inv_d23;
                s.Count = 2;
                s.V1 = s.V3;
                return;
            }

            // Must be in triangle123
            float inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
            s.V1.A = d123_1 * inv_d123;
            s.V2.A = d123_2 * inv_d123;
            s.V3.A = d123_3 * inv_d123;
            s.Count = 3;
        }

        public static DistanceOutput ShapeDistance(ref DistanceCache cache, in DistanceInput input, Simplex[]? simplexes, int simplexCapacity)
        {
            DistanceOutput output = new();

            ref readonly DistanceProxy proxyA = ref input.ProxyA;
            ref readonly DistanceProxy proxyB = ref input.ProxyB;

            Transform transformA = input.TransformA;
            Transform transformB = input.TransformB;

            // Initialize the simplex.
            Simplex simplex = MakeSimplexFromCache(cache, proxyA, transformA, proxyB, transformB);

            int simplexIndex = 0;
            if (simplexes != null && simplexIndex < simplexCapacity)
            {
                simplexes[simplexIndex] = simplex;
                simplexIndex += 1;
            }

            // Get simplex vertices as an array.
            var vertices = simplex.Vertices;
            const int k_maxIters = 20;

            // These store the vertices of the last simplex so that we can check for duplicates and prevent cycling.
            FixedArray3<int> saveA = new();
            FixedArray3<int> saveB = new();

            // Main iteration loop.
            int iter = 0;
            while (iter < k_maxIters)
            {
                // Copy simplex so we can identify duplicates.
                int saveCount = simplex.Count;
                for (int i = 0; i < saveCount; ++i)
                {
                    saveA[i] = vertices[i].IndexA;
                    saveB[i] = vertices[i].IndexB;
                }

                switch (simplex.Count)
                {
                case 1:
                    break;

                case 2:
                    SolveSimplex2(ref simplex);
                    break;

                case 3:
                    SolveSimplex3(ref simplex);
                    break;

                default:
                    Debug.Assert(false);
                    break;
                }

                // If we have 3 points, then the origin is in the corresponding triangle.
                if (simplex.Count == 3)
                {
                    break;
                }

                if (simplexes != null && simplexIndex < simplexCapacity)
                {
                    simplexes[simplexIndex] = simplex;
                    simplexIndex += 1;
                }

                // Get search direction.
                Vec2 d = ComputeSimplexSearchDirection(ref simplex);

                // Ensure the search direction is numerically fit.
                if (B2Math.Dot(d, d) < float.Epsilon * float.Epsilon)
                {
                    // The origin is probably contained by a line segment
                    // or triangle. Thus the shapes are overlapped.

                    // We can't return zero here even though there may be overlap.
                    // In case the simplex is a point, segment, or triangle it is difficult
                    // to determine if the origin is contained in the CSO or very close to it.
                    break;
                }

                // Compute a tentative new simplex vertex using support points.
                // support = support(b, d) - support(a, -d)
                ref SimplexVertex vertex = ref vertices[simplex.Count];
                vertex.IndexA = FindSupport(proxyA, B2Math.InvRotateVector(transformA.Q, B2Math.Neg(d)));
                vertex.WA = B2Math.TransformPoint(transformA, proxyA.Points[vertex.IndexA]);
                vertex.IndexB = FindSupport(proxyB, B2Math.InvRotateVector(transformB.Q, d));
                vertex.WB = B2Math.TransformPoint(transformB, proxyB.Points[vertex.IndexB]);
                vertex.W = B2Math.Sub(vertex.WB, vertex.WA);

                // Iteration count is equated to the number of support point calls.
                ++iter;

                // Check for duplicate support points. This is the main termination criteria.
                bool duplicate = false;
                for (int i = 0; i < saveCount; ++i)
                {
                    if (vertex.IndexA == saveA[i] && vertex.IndexB == saveB[i])
                    {
                        duplicate = true;
                        break;
                    }
                }

                // If we found a duplicate support point we must exit to avoid cycling.
                if (duplicate)
                {
                    break;
                }

                // New vertex is ok and needed.
                ++simplex.Count;
            }

            if (simplexes != null && simplexIndex < simplexCapacity)
            {
                simplexes[simplexIndex] = simplex;
                simplexIndex += 1;
            }

            // Prepare output
            ComputeSimplexWitnessPoints(out output.PointA, out output.PointB, ref simplex);
            output.Distance = B2Math.Distance(output.PointA, output.PointB);
            output.Iterations = iter;
            output.SimplexCount = simplexIndex;

            // Cache the simplex
            MakeSimplexCache(ref cache, ref simplex);

            // Apply radii if requested
            if (input.UseRadii)
            {
                if (output.Distance < float.Epsilon)
                {
                    // Shapes are too close to safely compute normal
                    Vec2 p = new Vec2(0.5f * (output.PointA.X + output.PointB.X), 0.5f * (output.PointA.Y + output.PointB.Y));
                    output.PointA = p;
                    output.PointB = p;
                    output.Distance = 0.0f;
                }
                else
                {
                    // Keep closest points on perimeter even if overlapped, this way
                    // the points move smoothly.
                    float rA = proxyA.Radius;
                    float rB = proxyB.Radius;
                    output.Distance = Math.Max(0.0f, output.Distance - rA - rB);
                    Vec2 normal = B2Math.Sub(output.PointB, output.PointA).Normalize;
                    Vec2 offsetA = new Vec2(rA * normal.X, rA * normal.Y);
                    Vec2 offsetB = new Vec2(rB * normal.X, rB * normal.Y);
                    output.PointA = B2Math.Add(output.PointA, offsetA);
                    output.PointB = B2Math.Sub(output.PointB, offsetB);
                }
            }

            return output;
        }

        /// <summary>
        /// GJK-raycast
        /// Algorithm by Gino van den Bergen.
        /// "Smooth Mesh Contacts with GJK" in Game Physics Pearls. 2010
        /// todo this is failing when used to raycast a box
        /// todo this converges slowly with a radius
        /// </summary>
        /// <param name="input"></param>
        /// <returns></returns>
        public static CastOutput ShapeCast(in ShapeCastPairInput input)
        {
            CastOutput output = new();
            output.Fraction = input.MaxFraction;

            ref readonly DistanceProxy proxyA = ref input.ProxyA;

            Transform xfA = input.TransformA;
            Transform xfB = input.TransformB;
            Transform xf = B2Math.InvMulTransforms(xfA, xfB);

            // Put proxyB in proxyA's frame to reduce round-off error
            DistanceProxy proxyB = new DistanceProxy();
            proxyB.Count = input.ProxyB.Count;
            proxyB.Radius = input.ProxyB.Radius;
            Debug.Assert(proxyB.Count <= Core.MaxPolygonVertices);

            for (int i = 0; i < proxyB.Count; ++i)
            {
                proxyB.Points[i] = B2Math.TransformPoint(xf, input.ProxyB.Points[i]);
            }

            float radius = proxyA.Radius + proxyB.Radius;

            Vec2 r = B2Math.RotateVector(xf.Q, input.TranslationB);
            float lambda = 0.0f;
            float maxFraction = input.MaxFraction;

            // Initial simplex
            Simplex simplex = new();
            simplex.Count = 0;

            // Get simplex vertices as an array.
            var vertices = simplex.Vertices;

            // Get an initial point in A - B
            int indexA = FindSupport(proxyA, B2Math.Neg(r));
            Vec2 wA = proxyA.Points[indexA];
            int indexB = FindSupport(proxyB, r);
            Vec2 wB = proxyB.Points[indexB];
            Vec2 v = B2Math.Sub(wA, wB);

            // Sigma is the target distance between proxies
            float linearSlop = Core.LinearSlop;
            float sigma = Math.Max(linearSlop, radius - linearSlop);

            // Main iteration loop.
            const int MaxIters = 20;
            int iter = 0;
            while (iter < MaxIters && v.Length > sigma + 0.5f * linearSlop)
            {
                Debug.Assert(simplex.Count < 3);

                output.Iterations += 1;

                // Support in direction -v (A - B)
                indexA = FindSupport(proxyA, B2Math.Neg(v));
                wA = proxyA.Points[indexA];
                indexB = FindSupport(proxyB, v);
                wB = proxyB.Points[indexB];
                Vec2 p = B2Math.Sub(wA, wB);

                // -v is a normal at p, normalize to work with sigma
                v = v.Normalize;

                // Intersect ray with plane
                float vp = B2Math.Dot(v, p);
                float vr = B2Math.Dot(v, r);
                if (vp - sigma > lambda * vr)
                {
                    if (vr <= 0.0f)
                    {
                        // miss
                        return output;
                    }

                    lambda = (vp - sigma) / vr;
                    if (lambda > maxFraction)
                    {
                        // too far
                        return output;
                    }

                    // reset the simplex
                    simplex.Count = 0;
                }

                // Reverse simplex since it works with B - A.
                // Shift by lambda * r because we want the closest point to the current clip point.
                // Note that the support point p is not shifted because we want the plane equation
                // to be formed in unshifted space.
                ref SimplexVertex vertex = ref vertices[simplex.Count];
                vertex.IndexA = indexB;
                vertex.WA = new Vec2(wB.X + lambda * r.X, wB.Y + lambda * r.Y);
                vertex.IndexB = indexA;
                vertex.WB = wA;
                vertex.W = B2Math.Sub(vertex.WB, vertex.WA);
                vertex.A = 1.0f;
                simplex.Count += 1;

                switch (simplex.Count)
                {
                case 1:
                    break;

                case 2:
                    SolveSimplex2(ref simplex);
                    break;

                case 3:
                    SolveSimplex3(ref simplex);
                    break;

                default:
                    Debug.Assert(false);
                    break;
                }

                // If we have 3 points, then the origin is in the corresponding triangle.
                if (simplex.Count == 3)
                {
                    // Overlap
                    return output;
                }

                // Get search direction.
                // todo use more accurate segment perpendicular
                v = ComputeSimplexClosestPoint(ref simplex);

                // Iteration count is equated to the number of support point calls.
                ++iter;
            }

            if (iter == 0 || lambda == 0.0f)
            {
                // Initial overlap
                return output;
            }

            // Prepare output.
            Vec2 pointA, pointB;
            ComputeSimplexWitnessPoints(out pointB, out pointA, ref simplex);

            Vec2 n = B2Math.Neg(v).Normalize;
            Vec2 point = new(pointA.X + proxyA.Radius * n.X, pointA.Y + proxyA.Radius * n.Y);

            output.Point = B2Math.TransformPoint(xfA, point);
            output.Normal = B2Math.RotateVector(xfA.Q, n);
            output.Fraction = lambda;
            output.Iterations = iter;
            output.Hit = true;
            return output;
        }

        // Warning: writing to these globals significantly slows multithreading performance
#if B2_TOI_DEBUG
        static float b2_toiTime, b2_toiMaxTime;

        static int b2_toiCalls, b2_toiIters, b2_toiMaxIters;

        static int b2_toiRootIters, b2_toiMaxRootIters;
#endif

        public enum SeparationType
        {
            PointsType,

            FaceAType,

            FaceBType
        }

        public struct SeparationFunction
        {
            public DistanceProxy ProxyA;

            public DistanceProxy ProxyB;

            public Sweep SweepA;

            public Sweep SweepB;

            public Vec2 LocalPoint;

            public Vec2 Axis;

            public SeparationType Type;
        }

        public static SeparationFunction MakeSeparationFunction(
            in DistanceCache cache,
            in DistanceProxy proxyA,
            in Sweep sweepA,
            in DistanceProxy proxyB,
            in Sweep sweepB,
            float t1)
        {
            SeparationFunction f = new();

            f.ProxyA = proxyA;
            f.ProxyB = proxyB;
            int count = cache.Count;
            Debug.Assert(0 < count && count < 3);

            f.SweepA = sweepA;
            f.SweepB = sweepB;

            Transform xfA = GetSweepTransform(sweepA, t1);
            Transform xfB = GetSweepTransform(sweepB, t1);

            if (count == 1)
            {
                f.Type = SeparationType.PointsType;
                Vec2 localPointA = proxyA.Points[cache.IndexA[0]];
                Vec2 localPointB = proxyB.Points[cache.IndexB[0]];
                Vec2 pointA = B2Math.TransformPoint(xfA, localPointA);
                Vec2 pointB = B2Math.TransformPoint(xfB, localPointB);
                f.Axis = B2Math.Sub(pointB, pointA).Normalize;
                f.LocalPoint = Vec2.Zero;
                return f;
            }

            if (cache.IndexA[0] == cache.IndexA[1])
            {
                // Two points on B and one on A.
                f.Type = SeparationType.FaceBType;
                Vec2 localPointB1 = proxyB.Points[cache.IndexB[0]];
                Vec2 localPointB2 = proxyB.Points[cache.IndexB[1]];

                f.Axis = B2Math.CrossVS(B2Math.Sub(localPointB2, localPointB1), 1.0f);
                f.Axis = f.Axis.Normalize;
                Vec2 normal = B2Math.RotateVector(xfB.Q, f.Axis);

                f.LocalPoint = new Vec2(0.5f * (localPointB1.X + localPointB2.X), 0.5f * (localPointB1.Y + localPointB2.Y));
                Vec2 pointB = B2Math.TransformPoint(xfB, f.LocalPoint);

                Vec2 localPointA = proxyA.Points[cache.IndexA[0]];
                Vec2 pointA = B2Math.TransformPoint(xfA, localPointA);

                float s = B2Math.Dot(B2Math.Sub(pointA, pointB), normal);
                if (s < 0.0f)
                {
                    f.Axis = B2Math.Neg(f.Axis);
                }

                return f;
            }

            {
                // Two points on A and one or two points on B.
                f.Type = SeparationType.FaceAType;
                Vec2 localPointA1 = proxyA.Points[cache.IndexA[0]];
                Vec2 localPointA2 = proxyA.Points[cache.IndexA[1]];

                f.Axis = B2Math.CrossVS(B2Math.Sub(localPointA2, localPointA1), 1.0f);
                f.Axis = f.Axis.Normalize;
                Vec2 normal = B2Math.RotateVector(xfA.Q, f.Axis);

                f.LocalPoint = new Vec2(0.5f * (localPointA1.X + localPointA2.X), 0.5f * (localPointA1.Y + localPointA2.Y));
                Vec2 pointA = B2Math.TransformPoint(xfA, f.LocalPoint);

                Vec2 localPointB = proxyB.Points[cache.IndexB[0]];
                Vec2 pointB = B2Math.TransformPoint(xfB, localPointB);

                float s = B2Math.Dot(B2Math.Sub(pointB, pointA), normal);
                if (s < 0.0f)
                {
                    f.Axis = B2Math.Neg(f.Axis);
                }

                return f;
            }
        }

        static float b2FindMinSeparation(in SeparationFunction f, out int indexA, out int indexB, float t)
        {
            Transform xfA = GetSweepTransform(f.SweepA, t);
            Transform xfB = GetSweepTransform(f.SweepB, t);

            switch (f.Type)
            {
            case SeparationType.PointsType:
            {
                Vec2 axisA = B2Math.InvRotateVector(xfA.Q, f.Axis);
                Vec2 axisB = B2Math.InvRotateVector(xfB.Q, B2Math.Neg(f.Axis));

                indexA = FindSupport(f.ProxyA, axisA);
                indexB = FindSupport(f.ProxyB, axisB);

                Vec2 localPointA = f.ProxyA.Points[indexA];
                Vec2 localPointB = f.ProxyB.Points[indexB];

                Vec2 pointA = B2Math.TransformPoint(xfA, localPointA);
                Vec2 pointB = B2Math.TransformPoint(xfB, localPointB);

                float separation = B2Math.Dot(B2Math.Sub(pointB, pointA), f.Axis);
                return separation;
            }

            case SeparationType.FaceAType:
            {
                Vec2 normal = B2Math.RotateVector(xfA.Q, f.Axis);
                Vec2 pointA = B2Math.TransformPoint(xfA, f.LocalPoint);

                Vec2 axisB = B2Math.InvRotateVector(xfB.Q, B2Math.Neg(normal));

                indexA = -1;
                indexB = FindSupport(f.ProxyB, axisB);

                Vec2 localPointB = f.ProxyB.Points[indexB];
                Vec2 pointB = B2Math.TransformPoint(xfB, localPointB);

                float separation = B2Math.Dot(B2Math.Sub(pointB, pointA), normal);
                return separation;
            }

            case SeparationType.FaceBType:
            {
                Vec2 normal = B2Math.RotateVector(xfB.Q, f.Axis);
                Vec2 pointB = B2Math.TransformPoint(xfB, f.LocalPoint);

                Vec2 axisA = B2Math.InvRotateVector(xfA.Q, B2Math.Neg(normal));

                indexB = -1;
                indexA = FindSupport(f.ProxyA, axisA);

                Vec2 localPointA = f.ProxyA.Points[indexA];
                Vec2 pointA = B2Math.TransformPoint(xfA, localPointA);

                float separation = B2Math.Dot(B2Math.Sub(pointA, pointB), normal);
                return separation;
            }

            default:
                Debug.Assert(false);
                indexA = -1;
                indexB = -1;
                return 0.0f;
            }
        }

        //
        public static float EvaluateSeparation(in SeparationFunction f, int indexA, int indexB, float t)
        {
            Transform xfA = GetSweepTransform(f.SweepA, t);
            Transform xfB = GetSweepTransform(f.SweepB, t);

            switch (f.Type)
            {
            case SeparationType.PointsType:
            {
                Vec2 localPointA = f.ProxyA.Points[indexA];
                Vec2 localPointB = f.ProxyB.Points[indexB];

                Vec2 pointA = B2Math.TransformPoint(xfA, localPointA);
                Vec2 pointB = B2Math.TransformPoint(xfB, localPointB);

                float separation = B2Math.Dot(B2Math.Sub(pointB, pointA), f.Axis);
                return separation;
            }

            case SeparationType.FaceAType:
            {
                Vec2 normal = B2Math.RotateVector(xfA.Q, f.Axis);
                Vec2 pointA = B2Math.TransformPoint(xfA, f.LocalPoint);

                Vec2 localPointB = f.ProxyB.Points[indexB];
                Vec2 pointB = B2Math.TransformPoint(xfB, localPointB);

                float separation = B2Math.Dot(B2Math.Sub(pointB, pointA), normal);
                return separation;
            }

            case SeparationType.FaceBType:
            {
                Vec2 normal = B2Math.RotateVector(xfB.Q, f.Axis);
                Vec2 pointB = B2Math.TransformPoint(xfB, f.LocalPoint);

                Vec2 localPointA = f.ProxyA.Points[indexA];
                Vec2 pointA = B2Math.TransformPoint(xfA, localPointA);

                float separation = B2Math.Dot(B2Math.Sub(pointA, pointB), normal);
                return separation;
            }

            default:
                Debug.Assert(false);
                return 0.0f;
            }
        }

        // CCD via the local separating axis method. This seeks progression
        // by computing the largest time at which separation is maintained.
        public static TOIOutput TimeOfImpact(in TOIInput input)
        {
#if B2_TOI_DEBUG
            var timer = Stopwatch.GetTimestamp();
            ++b2_toiCalls;
#endif

            TOIOutput output;
            output.State = TOIState.Unknown;
            output.T = input.TMax;

            ref readonly DistanceProxy proxyA = ref input.ProxyA;
            ref readonly DistanceProxy proxyB = ref input.ProxyB;

            ref readonly Sweep sweepA = ref input.SweepA;
            ref readonly Sweep sweepB = ref input.SweepB;
            Debug.Assert(sweepA.Q1.IsNormalized() && sweepA.Q2.IsNormalized());
            Debug.Assert(sweepB.Q1.IsNormalized() && sweepB.Q2.IsNormalized());

            float tMax = input.TMax;

            float totalRadius = proxyA.Radius + proxyB.Radius;
            float target = Math.Max(Core.LinearSlop, totalRadius - Core.LinearSlop);
            float tolerance = 0.25f * Core.LinearSlop;
            Debug.Assert(target > tolerance);

            float t1 = 0.0f;
            const int k_maxIterations = 20;
            int iter = 0;

            // Prepare input for distance query.
            DistanceCache cache = new();
            DistanceInput distanceInput;
            distanceInput.ProxyA = input.ProxyA;
            distanceInput.ProxyB = input.ProxyB;
            distanceInput.UseRadii = false;

            // The outer loop progressively attempts to compute new separating axes.
            // This loop terminates when an axis is repeated (no progress is made).
            for (;;)
            {
                Transform xfA = GetSweepTransform(sweepA, t1);
                Transform xfB = GetSweepTransform(sweepB, t1);

                // Get the distance between shapes. We can also use the results
                // to get a separating axis.
                distanceInput.TransformA = xfA;
                distanceInput.TransformB = xfB;
                DistanceOutput distanceOutput = ShapeDistance(ref cache, distanceInput, null, 0);

                // If the shapes are overlapped, we give up on continuous collision.
                if (distanceOutput.Distance <= 0.0f)
                {
                    // Failure!
                    output.State = TOIState.Overlapped;
                    output.T = 0.0f;
                    break;
                }

                if (distanceOutput.Distance < target + tolerance)
                {
                    // Victory!
                    output.State = TOIState.Hit;
                    output.T = t1;
                    break;
                }

                // Initialize the separating axis.
                SeparationFunction fcn = MakeSeparationFunction(cache, proxyA, sweepA, proxyB, sweepB, t1);

                // Compute the TOI on the separating axis. We do this by successively
                // resolving the deepest point. This loop is bounded by the number of vertices.
                bool done = false;
                float t2 = tMax;
                int pushBackIter = 0;
                for (;;)
                {
                    // Find the deepest point at t2. Store the witness point indices.
                    int indexA, indexB;
                    float s2 = b2FindMinSeparation(fcn, out indexA, out indexB, t2);

                    // Is the final configuration separated?
                    if (s2 > target + tolerance)
                    {
                        // Victory!
                        output.State = TOIState.Separated;
                        output.T = tMax;
                        done = true;
                        break;
                    }

                    // Has the separation reached tolerance?
                    if (s2 > target - tolerance)
                    {
                        // Advance the sweeps
                        t1 = t2;
                        break;
                    }

                    // Compute the initial separation of the witness points.
                    float s1 = EvaluateSeparation(fcn, indexA, indexB, t1);

                    // Check for initial overlap. This might happen if the root finder
                    // runs out of iterations.
                    if (s1 < target - tolerance)
                    {
                        output.State = TOIState.Failed;
                        output.T = t1;
                        done = true;
                        break;
                    }

                    // Check for touching
                    if (s1 <= target + tolerance)
                    {
                        // Victory! t1 should hold the TOI (could be 0.0).
                        output.State = TOIState.Hit;
                        output.T = t1;
                        done = true;
                        break;
                    }

                    // Compute 1D root of: f(x) - target = 0
                    int rootIterCount = 0;
                    float a1 = t1, a2 = t2;
                    for (;;)
                    {
                        // Use a mix of the secant rule and bisection.
                        float t;
                        if ((rootIterCount & 1) == 1)
                        {
                            // Secant rule to improve convergence.
                            t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
                        }
                        else
                        {
                            // Bisection to guarantee progress.
                            t = 0.5f * (a1 + a2);
                        }

                        ++rootIterCount;

#if B2_TOI_DEBUG
                        ++b2_toiRootIters;
#endif

                        float s = EvaluateSeparation(fcn, indexA, indexB, t);

                        if (Math.Abs(s - target) < tolerance)
                        {
                            // t2 holds a tentative value for t1
                            t2 = t;
                            break;
                        }

                        // Ensure we continue to bracket the root.
                        if (s > target)
                        {
                            a1 = t;
                            s1 = s;
                        }
                        else
                        {
                            a2 = t;
                            s2 = s;
                        }

                        if (rootIterCount == 50)
                        {
                            break;
                        }
                    }

#if B2_TOI_DEBUG
                    b2_toiMaxRootIters = Math.Max(b2_toiMaxRootIters, rootIterCount);
#endif

                    ++pushBackIter;

                    if (pushBackIter == Core.MaxPolygonVertices)
                    {
                        break;
                    }
                }

                ++iter;
#if B2_TOI_DEBUG
                ++b2_toiIters;
#endif

                if (done)
                {
                    break;
                }

                if (iter == k_maxIterations)
                {
                    // Root finder got stuck. Semi-victory.
                    output.State = TOIState.Failed;
                    output.T = t1;
                    break;
                }
            }

#if B2_TOI_DEBUG
            b2_toiMaxIters = Math.Max(b2_toiMaxIters, iter);

            var time = (Stopwatch.GetTimestamp() - timer) / 10000L;
            b2_toiMaxTime = Math.Max(b2_toiMaxTime, time);
            b2_toiTime += time;
#endif

            return output;
        }
    }
}