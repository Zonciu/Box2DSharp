using System;
using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;

namespace Box2DSharp.Collision
{
    /// A distance proxy is used by the GJK algorithm.
    /// It encapsulates any shape.
    public struct DistanceProxy
    {
        /// Initialize the proxy using the given shape. The shape
        /// must remain in scope while the proxy is in use.
        public void Set(Shape shape, int index)
        {
            switch (shape)
            {
            case CircleShape circle:
            {
                Vertices = new[] {circle.Position};
                Count = 1;
                Radius = circle.Radius;
            }
                break;

            case PolygonShape polygon:
            {
                Vertices = polygon.Vertices;
                Count = polygon.Count;
                Radius = polygon.Radius;
            }
                break;

            case ChainShape chain:
            {
                Debug.Assert(0 <= index && index < chain.Count);

                Buffer[0] = chain.Vertices[index];
                if (index + 1 < chain.Count)
                {
                    Buffer[1] = chain.Vertices[index + 1];
                }
                else
                {
                    Buffer[1] = chain.Vertices[0];
                }

                Vertices = Buffer;
                Count = 2;
                Radius = chain.Radius;
            }
                break;

            case EdgeShape edge:
            {
                Vertices = new[]
                {
                    edge.Vertex1,
                    edge.Vertex2
                };
                Count = 2;
                Radius = edge.Radius;
            }
                break;

            default:
                throw new NotSupportedException();
            }
        }

        /// Initialize the proxy using a vertex cloud and radius. The vertices
        /// must remain in scope while the proxy is in use.
        public void Set(Vector2[] vertices, int count, float radius)
        {
            Vertices = new Vector2[vertices.Length];
            Array.Copy(vertices, Vertices, vertices.Length);
            Count = count;
            Radius = radius;
        }

        /// Get the supporting vertex index in the given direction.
        public int GetSupport(in Vector2 d)
        {
            var bestIndex = 0;
            var bestValue = MathUtils.Dot(Vertices[0], d);
            for (var i = 1; i < Count; ++i)
            {
                var value = MathUtils.Dot(Vertices[i], d);
                if (value > bestValue)
                {
                    bestIndex = i;
                    bestValue = value;
                }
            }

            return bestIndex;
        }

        /// Get the supporting vertex in the given direction.
        public ref readonly Vector2 GetSupportVertex(in Vector2 d)
        {
            var bestIndex = 0;
            var bestValue = MathUtils.Dot(Vertices[0], d);
            for (var i = 1; i < Count; ++i)
            {
                var value = MathUtils.Dot(Vertices[i], d);
                if (value > bestValue)
                {
                    bestIndex = i;
                    bestValue = value;
                }
            }

            return ref Vertices[bestIndex];
        }

        /// Get the vertex count.
        public int GetVertexCount()
        {
            return Count;
        }

        /// Get a vertex by index. Used by b2Distance.
        public ref readonly Vector2 GetVertex(int index)
        {
            Debug.Assert(0 <= index && index < Count);
            return ref Vertices[index];
        }

        public Vector2[] Buffer;

        public Vector2[] Vertices;

        public int Count;

        public float Radius;
    }

    public static class DistanceAlgorithm
    {
        // GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
        public static int b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters;

        /// <summary>
        /// GJK碰撞检测
        /// </summary>
        /// <param name="output"></param>
        /// <param name="cache"></param>
        /// <param name="input"></param>
        public static void Distance(
            out DistanceOutput output,
            ref SimplexCache cache,
            in DistanceInput input)
        {
            ++b2_gjkCalls;
            output = new DistanceOutput();
            ref readonly var proxyA = ref input.ProxyA;
            ref readonly var proxyB = ref input.ProxyB;

            var transformA = input.TransformA;
            var transformB = input.TransformB;

            // Initialize the simplex.
            var simplex = new Simplex();
            simplex.ReadCache(
                ref cache,
                proxyA,
                transformA,
                proxyB,
                transformB);

            // Get simplex vertices as an array.
            ref var vertices = ref simplex.Vertices;
            const int maxIters = 20;

            // These store the vertices of the last simplex so that we
            // can check for duplicates and prevent cycling.
            var saveA = new int[3];
            var saveB = new int[3];
            var saveCount = 0;

            // Main iteration loop.
            var iter = 0;
            while (iter < maxIters)
            {
                // Copy simplex so we can identify duplicates.
                saveCount = simplex.Count;
                for (var i = 0; i < saveCount; ++i)
                {
                    saveA[i] = vertices.GetRef(i).IndexA;
                    saveB[i] = vertices.GetRef(i).IndexB;
                }

                switch (simplex.Count)
                {
                case 1:
                    break;

                case 2:
                    simplex.Solve2();
                    break;

                case 3:
                    simplex.Solve3();
                    break;

                default:
                    throw new ArgumentOutOfRangeException(nameof(simplex.Count));
                }

                // If we have 3 points, then the origin is in the corresponding triangle.
                if (simplex.Count == 3)
                {
                    break;
                }

                // Get search direction.
                var d = simplex.GetSearchDirection();

                // Ensure the search direction is numerically fit.
                if (d.LengthSquared() < Settings.Epsilon * Settings.Epsilon)
                {
                    // The origin is probably contained by a line segment
                    // or triangle. Thus the shapes are overlapped.

                    // We can't return zero here even though there may be overlap.
                    // In case the simplex is a point, segment, or triangle it is difficult
                    // to determine if the origin is contained in the CSO or very close to it.
                    break;
                }

                // Compute a tentative new simplex vertex using support points.
                ref var vertex = ref vertices.GetRef(simplex.Count);
                vertex.IndexA = proxyA.GetSupport(MathUtils.MulT(transformA.Rotation, -d));
                vertex.Wa = MathUtils.Mul(transformA, proxyA.GetVertex(vertex.IndexA));

                vertex.IndexB = proxyB.GetSupport(MathUtils.MulT(transformB.Rotation, d));
                vertex.Wb = MathUtils.Mul(transformB, proxyB.GetVertex(vertex.IndexB));
                vertex.W = vertex.Wb - vertex.Wa;

                // Iteration count is equated to the number of support point calls.
                ++iter;
                ++b2_gjkIters;

                // Check for duplicate support points. This is the main termination criteria.
                var duplicate = false;
                for (var i = 0; i < saveCount; ++i)
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

            b2_gjkMaxIters = Math.Max(b2_gjkMaxIters, iter);

            // Prepare output.
            simplex.GetWitnessPoints(out output.PointA, out output.PointB);
            output.Distance = MathUtils.Distance(output.PointA, output.PointB);
            output.Iterations = iter;

            // Cache the simplex.
            simplex.WriteCache(ref cache);

            // Apply radii if requested.
            if (input.UseRadii)
            {
                var rA = proxyA.Radius;
                var rB = proxyB.Radius;

                if (output.Distance > rA + rB && output.Distance > Settings.Epsilon)
                {
                    // Shapes are still no overlapped.
                    // Move the witness points to the outer surface.
                    output.Distance -= rA + rB;
                    var normal = output.PointB - output.PointA;
                    normal.Normalize();
                    output.PointA += rA * normal;
                    output.PointB -= rB * normal;
                }
                else
                {
                    // Shapes are overlapped when radii are considered.
                    // Move the witness points to the middle.
                    var p = 0.5f * (output.PointA + output.PointB);
                    output.PointA = p;
                    output.PointB = p;
                    output.Distance = 0.0f;
                }
            }
        }

        public static bool ShapeCast(out ShapeCastOutput output, in ShapeCastInput input)
        {
            output = new ShapeCastOutput
            {
                Iterations = 0,
                Lambda = 1.0f,
                Normal = Vector2.Zero,
                Point = Vector2.Zero
            };

            ref readonly var proxyA = ref input.ProxyA;
            ref readonly var proxyB = ref input.ProxyB;

            var radiusA = Math.Max(proxyA.Radius, Settings.PolygonRadius);
            var radiusB = Math.Max(proxyB.Radius, Settings.PolygonRadius);
            var radius = radiusA + radiusB;

            var xfA = input.TransformA;
            var xfB = input.TransformB;

            var r = input.TranslationB;
            var n = new Vector2(0.0f, 0.0f);
            var lambda = 0.0f;

            // Initial simplex
            var simplex = new Simplex {Count = 0};

            // Get simplex vertices as an array.
            // ref var vertices = ref simplex.Vertices;

            // Get support point in -r direction
            var indexA = proxyA.GetSupport(MathUtils.MulT(xfA.Rotation, -r));
            var wA = MathUtils.Mul(xfA, proxyA.GetVertex(indexA));
            var indexB = proxyB.GetSupport(MathUtils.MulT(xfB.Rotation, r));
            var wB = MathUtils.Mul(xfB, proxyB.GetVertex(indexB));
            var v = wA - wB;

            // Sigma is the target distance between polygons
            var sigma = Math.Max(Settings.PolygonRadius, radius - Settings.PolygonRadius);
            const float tolerance = 0.5f * Settings.LinearSlop;

            // Main iteration loop.
            // 迭代次数上限
            const int maxIters = 20;
            var iter = 0;
            while (iter < maxIters && Math.Abs(v.Length() - sigma) > tolerance)
            {
                Debug.Assert(simplex.Count < 3);

                output.Iterations += 1;

                // Support in direction -v (A - B)
                indexA = proxyA.GetSupport(MathUtils.MulT(xfA.Rotation, -v));
                wA = MathUtils.Mul(xfA, proxyA.GetVertex(indexA));
                indexB = proxyB.GetSupport(MathUtils.MulT(xfB.Rotation, v));
                wB = MathUtils.Mul(xfB, proxyB.GetVertex(indexB));
                var p = wA - wB;

                // -v is a normal at p
                v.Normalize();

                // Intersect ray with plane
                var vp = MathUtils.Dot(v, p);
                var vr = MathUtils.Dot(v, r);
                if (vp - sigma > lambda * vr)
                {
                    if (vr <= 0.0f)
                    {
                        return false;
                    }

                    lambda = (vp - sigma) / vr;
                    if (lambda > 1.0f)
                    {
                        return false;
                    }

                    n = -v;
                    simplex.Count = 0;
                }

                // Reverse simplex since it works with B - A.
                // Shift by lambda * r because we want the closest point to the current clip point.
                // Note that the support point p is not shifted because we want the plane equation
                // to be formed in unshifted space.
                ref var vertex = ref simplex.Vertices.GetRef(simplex.Count);
                vertex.IndexA = indexB;
                vertex.Wa = wB + lambda * r;
                vertex.IndexB = indexA;
                vertex.Wb = wA;
                vertex.W = vertex.Wb - vertex.Wa;
                vertex.A = 1.0f;
                simplex.Count += 1;

                switch (simplex.Count)
                {
                case 1:
                    break;

                case 2:
                    simplex.Solve2();
                    break;

                case 3:
                    simplex.Solve3();
                    break;

                default:
                    Debug.Assert(false);
                    break;
                }

                // If we have 3 points, then the origin is in the corresponding triangle.
                if (simplex.Count == 3)
                {
                    // Overlap
                    return false;
                }

                // Get search direction.
                v = simplex.GetClosestPoint();

                // Iteration count is equated to the number of support point calls.
                ++iter;
            }

            // Prepare output.
            //Vector2 pointA, pointB;
            simplex.GetWitnessPoints(out var pointB, out var pointA);

            if (v.LengthSquared() > 0.0f)
            {
                n = -v;
                n.Normalize();
            }

            output.Point = pointA + radiusA * n;
            output.Normal = n;
            output.Lambda = lambda;
            output.Iterations = iter;
            return true;
        }
    }
}