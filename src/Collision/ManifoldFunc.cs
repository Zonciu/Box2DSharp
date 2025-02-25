using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.Intrinsics;

namespace Box2DSharp
{
    public static class ManifoldFunc
    {
        public static ushort MakeId(int a, int b)
        {
            return (ushort)(((byte)a << 8) | (byte)b);
        }

        private static Polygon MakeCapsule(Vec2 p1, Vec2 p2, float radius)
        {
            Polygon shape = new();
            shape.Vertices[0] = p1;
            shape.Vertices[1] = p2;
            shape.Centroid = B2Math.Lerp(p1, p2, 0.5f);

            Vec2 d = B2Math.Sub(p2, p1);
            Debug.Assert(B2Math.LengthSquared(d) > float.Epsilon);
            Vec2 axis = d.Normalize;
            Vec2 normal = B2Math.RightPerp(axis);

            shape.Normals[0] = normal;
            shape.Normals[1] = B2Math.Neg(normal);
            shape.Count = 2;
            shape.Radius = radius;

            return shape;
        }

        // point = qA * localAnchorA + pA
        // localAnchorB = qBc * (point - pB)
        // anchorB = point - pB = qA * localAnchorA + pA - pB
        //         = anchorA + (pA - pB)
        public static Manifold CollideCircles(in Circle circleA, in Transform xfA, in Circle circleB, in Transform xfB)
        {
            Manifold manifold = new();

            Transform xf = B2Math.InvMulTransforms(xfA, xfB);

            Vec2 pointA = circleA.Center;
            Vec2 pointB = B2Math.TransformPoint(xf, circleB.Center);

            var (normal, distance) = B2Math.Sub(pointB, pointA).GetLengthAndNormalize();

            float radiusA = circleA.Radius;
            float radiusB = circleB.Radius;

            float separation = distance - radiusA - radiusB;
            if (separation > Core.SpeculativeDistance)
            {
                return manifold;
            }

            Vec2 cA = B2Math.MulAdd(pointA, radiusA, normal);
            Vec2 cB = B2Math.MulAdd(pointB, -radiusB, normal);
            Vec2 contactPointA = B2Math.Lerp(cA, cB, 0.5f);

            manifold.Normal = B2Math.RotateVector(xfA.Q, normal);
            ref ManifoldPoint mp = ref manifold.Point1;
            mp.AnchorA = B2Math.RotateVector(xfA.Q, contactPointA);
            mp.AnchorB = B2Math.Add(mp.AnchorA, B2Math.Sub(xfA.P, xfB.P));
            mp.Point = B2Math.Add(xfA.P, mp.AnchorA);
            mp.Separation = separation;
            mp.Id = 0;
            manifold.PointCount = 1;
            return manifold;
        }

        /// Compute the collision manifold between a capsule and circle
        public static Manifold CollideCapsuleAndCircle(in Capsule capsuleA, in Transform xfA, in Circle circleB, in Transform xfB)
        {
            Manifold manifold = new();

            Transform xf = B2Math.InvMulTransforms(xfA, xfB);

            // Compute circle position in the frame of the capsule.
            Vec2 pB = B2Math.TransformPoint(xf, circleB.Center);

            // Compute closest point
            Vec2 p1 = capsuleA.Center1;
            Vec2 p2 = capsuleA.Center2;

            Vec2 e = B2Math.Sub(p2, p1);

            // dot(p - pA, e) = 0
            // dot(p - (p1 + s1 * e), e) = 0
            // s1 = dot(p - p1, e)
            Vec2 pA;
            float s1 = B2Math.Dot(B2Math.Sub(pB, p1), e);
            float s2 = B2Math.Dot(B2Math.Sub(p2, pB), e);
            if (s1 < 0.0f)
            {
                // p1 region
                pA = p1;
            }
            else if (s2 < 0.0f)
            {
                // p2 region
                pA = p2;
            }
            else
            {
                // circle colliding with segment interior
                float s = s1 / B2Math.Dot(e, e);
                pA = B2Math.MulAdd(p1, s, e);
            }

            var (normal, distance) = B2Math.Sub(pB, pA).GetLengthAndNormalize();

            float radiusA = capsuleA.Radius;
            float radiusB = circleB.Radius;
            float separation = distance - radiusA - radiusB;
            if (separation > Core.SpeculativeDistance)
            {
                return manifold;
            }

            Vec2 cA = B2Math.MulAdd(pA, radiusA, normal);
            Vec2 cB = B2Math.MulAdd(pB, -radiusB, normal);
            Vec2 contactPointA = B2Math.Lerp(cA, cB, 0.5f);

            manifold.Normal = B2Math.RotateVector(xfA.Q, normal);
            ref ManifoldPoint mp = ref manifold.Point1;
            mp.AnchorA = B2Math.RotateVector(xfA.Q, contactPointA);
            mp.AnchorB = B2Math.Add(mp.AnchorA, B2Math.Sub(xfA.P, xfB.P));
            mp.Point = B2Math.Add(xfA.P, mp.AnchorA);
            mp.Separation = separation;
            mp.Id = 0;
            manifold.PointCount = 1;
            return manifold;
        }

        public static Manifold CollidePolygonAndCircle(in Polygon polygonA, in Transform xfA, in Circle circleB, in Transform xfB)
        {
            Manifold manifold = new();
            float speculativeDistance = Core.SpeculativeDistance;

            Transform xf = B2Math.InvMulTransforms(xfA, xfB);

            // Compute circle position in the frame of the polygon.
            Vec2 c = B2Math.TransformPoint(xf, circleB.Center);
            float radiusA = polygonA.Radius;
            float radiusB = circleB.Radius;
            float radius = radiusA + radiusB;

            // Find the min separating edge.
            int normalIndex = 0;
            float separation = -float.MaxValue;
            int vertexCount = polygonA.Count;
            ref readonly var vertices = ref polygonA.Vertices;
            ref readonly var normals = ref polygonA.Normals;

            for (int i = 0; i < vertexCount; ++i)
            {
                float s = B2Math.Dot(normals[i], B2Math.Sub(c, vertices[i]));
                if (s > separation)
                {
                    separation = s;
                    normalIndex = i;
                }
            }

            if (separation > radius + speculativeDistance)
            {
                return manifold;
            }

            // Vertices of the reference edge.
            int vertIndex1 = normalIndex;
            int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
            Vec2 v1 = vertices[vertIndex1];
            Vec2 v2 = vertices[vertIndex2];

            // Compute barycentric coordinates
            float u1 = B2Math.Dot(B2Math.Sub(c, v1), B2Math.Sub(v2, v1));
            float u2 = B2Math.Dot(B2Math.Sub(c, v2), B2Math.Sub(v1, v2));

            if (u1 < 0.0f && separation > float.Epsilon)
            {
                // Circle center is closest to v1 and safely outside the polygon
                Vec2 normal = B2Math.Sub(c, v1).Normalize;
                separation = B2Math.Dot(B2Math.Sub(c, v1), normal);
                if (separation > radius + speculativeDistance)
                {
                    return manifold;
                }

                Vec2 cA = B2Math.MulAdd(v1, radiusA, normal);
                Vec2 cB = B2Math.MulSub(c, radiusB, normal);
                Vec2 contactPointA = B2Math.Lerp(cA, cB, 0.5f);

                manifold.Normal = B2Math.RotateVector(xfA.Q, normal);
                ref ManifoldPoint mp = ref manifold.Point1;
                mp.AnchorA = B2Math.RotateVector(xfA.Q, contactPointA);
                mp.AnchorB = B2Math.Add(mp.AnchorA, B2Math.Sub(xfA.P, xfB.P));
                mp.Point = B2Math.Add(xfA.P, mp.AnchorA);
                mp.Separation = B2Math.Dot(B2Math.Sub(cB, cA), normal);
                mp.Id = 0;
                manifold.PointCount = 1;
            }
            else if (u2 < 0.0f && separation > float.Epsilon)
            {
                // Circle center is closest to v2 and safely outside the polygon
                Vec2 normal = B2Math.Sub(c, v2).Normalize;
                separation = B2Math.Dot(B2Math.Sub(c, v2), normal);
                if (separation > radius + speculativeDistance)
                {
                    return manifold;
                }

                Vec2 cA = B2Math.MulAdd(v2, radiusA, normal);
                Vec2 cB = B2Math.MulSub(c, radiusB, normal);
                Vec2 contactPointA = B2Math.Lerp(cA, cB, 0.5f);

                manifold.Normal = B2Math.RotateVector(xfA.Q, normal);
                ref ManifoldPoint mp = ref manifold.Point1;
                mp.AnchorA = B2Math.RotateVector(xfA.Q, contactPointA);
                mp.AnchorB = B2Math.Add(mp.AnchorA, B2Math.Sub(xfA.P, xfB.P));
                mp.Point = B2Math.Add(xfA.P, mp.AnchorA);
                mp.Separation = B2Math.Dot(B2Math.Sub(cB, cA), normal);
                mp.Id = 0;
                manifold.PointCount = 1;
            }
            else
            {
                // Circle center is between v1 and v2. Center may be inside polygon
                Vec2 normal = normals[normalIndex];
                manifold.Normal = B2Math.RotateVector(xfA.Q, normal);

                // cA is the projection of the circle center onto to the reference edge
                Vec2 cA = B2Math.MulAdd(c, radiusA - B2Math.Dot(B2Math.Sub(c, v1), normal), normal);

                // cB is the deepest point on the circle with respect to the reference edge
                Vec2 cB = B2Math.MulSub(c, radiusB, normal);

                Vec2 contactPointA = B2Math.Lerp(cA, cB, 0.5f);

                // The contact point is the midpoint in world space
                ref ManifoldPoint mp = ref manifold.Point1;
                mp.AnchorA = B2Math.RotateVector(xfA.Q, contactPointA);
                mp.AnchorB = B2Math.Add(mp.AnchorA, B2Math.Sub(xfA.P, xfB.P));
                mp.Point = B2Math.Add(xfA.P, mp.AnchorA);
                mp.Separation = separation - radius;
                mp.Id = 0;
                manifold.PointCount = 1;
            }

            return manifold;
        }

        public static Manifold CollideCapsules(in Capsule capsuleA, in Transform xfA, in Capsule capsuleB, in Transform xfB)
        {
            Polygon polyA = MakeCapsule(capsuleA.Center1, capsuleA.Center2, capsuleA.Radius);
            Polygon polyB = MakeCapsule(capsuleB.Center1, capsuleB.Center2, capsuleB.Radius);
            return CollidePolygons(polyA, xfA, polyB, xfB);
        }

        public static Manifold CollideSegmentAndCapsule(in Segment segmentA, in Transform xfA, in Capsule capsuleB, in Transform xfB)
        {
            Polygon polyA = MakeCapsule(segmentA.Point1, segmentA.Point2, 0.0f);
            Polygon polyB = MakeCapsule(capsuleB.Center1, capsuleB.Center2, capsuleB.Radius);
            return CollidePolygons(polyA, xfA, polyB, xfB);
        }

        public static Manifold CollidePolygonAndCapsule(in Polygon polygonA, in Transform xfA, in Capsule capsuleB, in Transform xfB)
        {
            Polygon polyB = MakeCapsule(capsuleB.Center1, capsuleB.Center2, capsuleB.Radius);
            return CollidePolygons(polygonA, xfA, polyB, xfB);
        }

        // Polygon clipper used to compute contact points when there are potentially two contact points.
        public static Manifold ClipPolygons(in Polygon polyA, in Polygon polyB, int edgeA, int edgeB, bool flip)
        {
            Manifold manifold = new();

            // reference polygon
            ref readonly Polygon poly1 = ref polyA;
            int i11, i12;

            // incident polygon
            ref readonly Polygon poly2 = ref polyB;
            int i21, i22;

            if (flip)
            {
                poly1 = ref polyB;
                poly2 = ref polyA;
                i11 = edgeB;
                i12 = edgeB + 1 < polyB.Count ? edgeB + 1 : 0;
                i21 = edgeA;
                i22 = edgeA + 1 < polyA.Count ? edgeA + 1 : 0;
            }
            else
            {
                poly1 = ref polyA;
                poly2 = ref polyB;
                i11 = edgeA;
                i12 = edgeA + 1 < polyA.Count ? edgeA + 1 : 0;
                i21 = edgeB;
                i22 = edgeB + 1 < polyB.Count ? edgeB + 1 : 0;
            }

            Vec2 normal = poly1.Normals[i11];

            // Reference edge vertices
            Vec2 v11 = poly1.Vertices[i11];
            Vec2 v12 = poly1.Vertices[i12];

            // Incident edge vertices
            Vec2 v21 = poly2.Vertices[i21];
            Vec2 v22 = poly2.Vertices[i22];

            Vec2 tangent = B2Math.CrossSV(1.0f, normal);

            float lower1 = 0.0f;
            float upper1 = B2Math.Dot(B2Math.Sub(v12, v11), tangent);

            // Incident edge points opposite of tangent due to CCW winding
            float upper2 = B2Math.Dot(B2Math.Sub(v21, v11), tangent);
            float lower2 = B2Math.Dot(B2Math.Sub(v22, v11), tangent);

            // This check can fail slightly due to mismatch with GJK code.
            // Perhaps fall back to a single point here? Otherwise we get two coincident points.
            // if (upper2 < lower1 || upper1 < lower2)
            //{
            //	// numeric failure
            //	Debug.Assert(false);
            //	return manifold;
            //}

            Vec2 vLower;
            if (lower2 < lower1 && upper2 - lower2 > float.Epsilon)
            {
                vLower = B2Math.Lerp(v22, v21, (lower1 - lower2) / (upper2 - lower2));
            }
            else
            {
                vLower = v22;
            }

            Vec2 vUpper;
            if (upper2 > upper1 && upper2 - lower2 > float.Epsilon)
            {
                vUpper = B2Math.Lerp(v22, v21, (upper1 - lower2) / (upper2 - lower2));
            }
            else
            {
                vUpper = v21;
            }

            // todo vLower can be very close to vUpper, reduce to one point?

            float separationLower = B2Math.Dot(B2Math.Sub(vLower, v11), normal);
            float separationUpper = B2Math.Dot(B2Math.Sub(vUpper, v11), normal);

            float r1 = poly1.Radius;
            float r2 = poly2.Radius;

            // Put contact points at midpoint, accounting for radii
            vLower = B2Math.MulAdd(vLower, 0.5f * (r1 - r2 - separationLower), normal);
            vUpper = B2Math.MulAdd(vUpper, 0.5f * (r1 - r2 - separationUpper), normal);

            float radius = r1 + r2;

            if (flip == false)
            {
                manifold.Normal = normal;

                {
                    ref ManifoldPoint cp = ref manifold.Point1;
                    cp.AnchorA = vLower;
                    cp.Separation = separationLower - radius;
                    cp.Id = MakeId(i11, i22);
                    manifold.PointCount += 1;
                }

                {
                    ref ManifoldPoint cp = ref manifold.Point2;
                    cp.AnchorA = vUpper;
                    cp.Separation = separationUpper - radius;
                    cp.Id = MakeId(i12, i21);
                    manifold.PointCount += 1;
                }
            }
            else
            {
                manifold.Normal = B2Math.Neg(normal);

                {
                    ref ManifoldPoint cp = ref manifold.Point1;
                    cp.AnchorA = vUpper;
                    cp.Separation = separationUpper - radius;
                    cp.Id = MakeId(i21, i12);
                    manifold.PointCount += 1;
                }

                {
                    ref ManifoldPoint cp = ref manifold.Point2;
                    cp.AnchorA = vLower;
                    cp.Separation = separationLower - radius;
                    cp.Id = MakeId(i22, i11);
                    manifold.PointCount += 1;
                }
            }

            return manifold;
        }

        /// <summary>
        /// Find the max separation between poly1 and poly2 using edge normals from poly1.
        /// </summary>
        /// <param name="edgeIndex"></param>
        /// <param name="poly1"></param>
        /// <param name="poly2"></param>
        /// <returns></returns>
        private static float FindMaxSeparation(out int edgeIndex, ref Polygon poly1, ref Polygon poly2)
        {
            int count1 = poly1.Count;
            int count2 = poly2.Count;
            ref readonly var n1s = ref poly1.Normals;
            ref readonly var v1s = ref poly1.Vertices;
            ref readonly var v2s = ref poly2.Vertices;
            Span<float> v2sXSpan = stackalloc float[Vector<float>.Count];
            Span<float> v2sYSpan = stackalloc float[Vector<float>.Count];
            for (int i = 0; i < count2; i++)
            {
                v2sXSpan[i] = v2s[i].X;
                v2sYSpan[i] = v2s[i].Y;
            }

            Vector<float> v2sX = new Vector<float>(v2sXSpan);
            Vector<float> v2sY = new Vector<float>(v2sYSpan);

            int bestIndex = 0;
            float maxSeparation = -float.MaxValue;
            for (int i = 0; i < count1; ++i)
            {
                // Get poly1 normal in frame2.
                Vec2 n = n1s[i];
                Vec2 v1 = v1s[i];

                // Find the deepest point for normal i.
                Vector<float> v1X = new Vector<float>(v1.X);
                Vector<float> v1Y = new Vector<float>(v1.Y);
                var ij = n.X * (v2sX - v1X) + n.Y * (v2sY - v1Y);

                float si = float.MaxValue;
                for (int j = 0; j < count2; ++j)
                {
                    float sij = ij[j];
                    if (sij < si)
                    {
                        si = sij;
                    }
                }

                if (si > maxSeparation)
                {
                    maxSeparation = si;
                    bestIndex = i;
                }
            }

            edgeIndex = bestIndex;
            return maxSeparation;
        }

        /// <summary>
        /// Due to speculation, every polygon is rounded
        /// Algorithm:
        ///
        /// compute edge separation using the separating axis test (SAT)
        /// if (separation > speculation_distance)
        ///   return
        /// find reference and incident edge
        /// if separation >= 0.1f * b2_linearSlop
        ///   compute closest points between reference and incident edge
        ///   if vertices are closest
        ///      single vertex-vertex contact
        ///   else
        ///      clip edges
        ///   end
        /// else
        ///   clip edges
        /// end
        /// </summary>
        /// <param name="polygonA"></param>
        /// <param name="xfA"></param>
        /// <param name="polygonB"></param>
        /// <param name="xfB"></param>
        /// <returns></returns>
        public static Manifold CollidePolygons(in Polygon polygonA, in Transform xfA, in Polygon polygonB, in Transform xfB)
        {
            Vec2 origin = polygonA.Vertices[0];

            // Shift polyA to origin
            // pw = q * pb + p
            // pw = q * (pbs + origin) + p
            // pw = q * pbs + (p + q * origin)
            Transform sfA = new(B2Math.Add(xfA.P, B2Math.RotateVector(xfA.Q, origin)), xfA.Q);
            Transform xf = B2Math.InvMulTransforms(sfA, xfB);

            Polygon localPolyA = new()
            {
                Count = polygonA.Count,
                Radius = polygonA.Radius
            };
            localPolyA.Vertices.V0 = Vec2.Zero;
            localPolyA.Normals.V0 = polygonA.Normals[0];
            for (int i = 1; i < localPolyA.Count; ++i)
            {
                localPolyA.Vertices[i] = B2Math.Sub(polygonA.Vertices[i], origin);
                localPolyA.Normals[i] = polygonA.Normals[i];
            }

            // Put polyB in polyA's frame to reduce round-off error
            Polygon localPolyB = new()
            {
                Count = polygonB.Count,
                Radius = polygonB.Radius
            };
            for (int i = 0; i < localPolyB.Count; ++i)
            {
                localPolyB.Vertices[i] = B2Math.TransformPoint(xf, polygonB.Vertices[i]);
                localPolyB.Normals[i] = B2Math.RotateVector(xf.Q, polygonB.Normals[i]);
            }

            float separationA = FindMaxSeparation(out var edgeA, ref localPolyA, ref localPolyB);
            float separationB = FindMaxSeparation(out var edgeB, ref localPolyB, ref localPolyA);

            float radius = localPolyA.Radius + localPolyB.Radius;

            if (separationA > Core.SpeculativeDistance + radius || separationB > Core.SpeculativeDistance + radius)
            {
                return new Manifold();
            }

            // Find incident edge
            bool flip;
            if (separationA >= separationB)
            {
                flip = false;

                Vec2 searchDirection = localPolyA.Normals[edgeA];

                // Find the incident edge on polyB
                int count = localPolyB.Count;
                ref var normals = ref localPolyB.Normals;
                edgeB = 0;
                float minDot = float.MaxValue;
                for (int i = 0; i < count; ++i)
                {
                    float dot = B2Math.Dot(searchDirection, normals[i]);
                    if (dot < minDot)
                    {
                        minDot = dot;
                        edgeB = i;
                    }
                }
            }
            else
            {
                flip = true;

                ref readonly Vec2 searchDirection = ref localPolyB.Normals[edgeB];

                // Find the incident edge on polyA
                int count = localPolyA.Count;
                ref var normals = ref localPolyA.Normals;
                edgeA = 0;
                float minDot = float.MaxValue;
                for (int i = 0; i < count; ++i)
                {
                    float dot = B2Math.Dot(searchDirection, normals[i]);
                    if (dot < minDot)
                    {
                        minDot = dot;
                        edgeA = i;
                    }
                }
            }

            Manifold manifold = new();

            // Using slop here to ensure vertex-vertex normal vectors can be safely normalized
            // todo this means edge clipping needs to handle slightly non-overlapping edges.
            if (separationA > 0.1f * Core.LinearSlop || separationB > 0.1f * Core.LinearSlop)
            {
                // Polygons are disjoint. Find closest points between reference edge and incident edge
                // Reference edge on polygon A
                int i11 = edgeA;
                int i12 = edgeA + 1 < localPolyA.Count ? edgeA + 1 : 0;
                int i21 = edgeB;
                int i22 = edgeB + 1 < localPolyB.Count ? edgeB + 1 : 0;

                Vec2 v11 = localPolyA.Vertices[i11];
                Vec2 v12 = localPolyA.Vertices[i12];
                Vec2 v21 = localPolyB.Vertices[i21];
                Vec2 v22 = localPolyB.Vertices[i22];

                SegmentDistanceResult result = DistanceFunc.SegmentDistance(v11, v12, v21, v22);

                if (result is { Fraction1: 0.0f, Fraction2: 0.0f })
                {
                    // v11 - v21
                    Vec2 normal = B2Math.Sub(v21, v11);
                    Debug.Assert(result.DistanceSquared > 0.0f);
                    float distance = MathF.Sqrt(result.DistanceSquared);
                    if (distance > Core.SpeculativeDistance + radius)
                    {
                        return manifold;
                    }

                    float invDistance = 1.0f / distance;
                    normal.X *= invDistance;
                    normal.Y *= invDistance;

                    Vec2 c1 = B2Math.MulAdd(v11, localPolyA.Radius, normal);
                    Vec2 c2 = B2Math.MulAdd(v21, -localPolyB.Radius, normal);

                    manifold.Normal = normal;
                    manifold.Point1.AnchorA = B2Math.Lerp(c1, c2, 0.5f);
                    manifold.Point1.Separation = distance - radius;
                    manifold.Point1.Id = MakeId(i11, i21);
                    manifold.PointCount = 1;
                }
                else if (result is { Fraction1: 0.0f, Fraction2: 1.0f })
                {
                    // v11 - v22
                    Vec2 normal = B2Math.Sub(v22, v11);
                    Debug.Assert(result.DistanceSquared > 0.0f);
                    float distance = MathF.Sqrt(result.DistanceSquared);
                    if (distance > Core.SpeculativeDistance + radius)
                    {
                        return manifold;
                    }

                    float invDistance = 1.0f / distance;
                    normal.X *= invDistance;
                    normal.Y *= invDistance;

                    Vec2 c1 = B2Math.MulAdd(v11, localPolyA.Radius, normal);
                    Vec2 c2 = B2Math.MulAdd(v22, -localPolyB.Radius, normal);

                    manifold.Normal = normal;
                    manifold.Point1.AnchorA = B2Math.Lerp(c1, c2, 0.5f);
                    manifold.Point1.Separation = distance - radius;
                    manifold.Point1.Id = MakeId(i11, i22);
                    manifold.PointCount = 1;
                }
                else if (result is { Fraction1: 1.0f, Fraction2: 0.0f })
                {
                    // v12 - v21
                    Vec2 normal = B2Math.Sub(v21, v12);
                    Debug.Assert(result.DistanceSquared > 0.0f);
                    float distance = MathF.Sqrt(result.DistanceSquared);
                    if (distance > Core.SpeculativeDistance + radius)
                    {
                        return manifold;
                    }

                    float invDistance = 1.0f / distance;
                    normal.X *= invDistance;
                    normal.Y *= invDistance;

                    Vec2 c1 = B2Math.MulAdd(v12, localPolyA.Radius, normal);
                    Vec2 c2 = B2Math.MulAdd(v21, -localPolyB.Radius, normal);

                    manifold.Normal = normal;
                    manifold.Point1.AnchorA = B2Math.Lerp(c1, c2, 0.5f);
                    manifold.Point1.Separation = distance - radius;
                    manifold.Point1.Id = MakeId(i12, i21);
                    manifold.PointCount = 1;
                }
                else if (result is { Fraction1: 1.0f, Fraction2: 1.0f })
                {
                    // v12 - v22
                    Vec2 normal = B2Math.Sub(v22, v12);
                    Debug.Assert(result.DistanceSquared > 0.0f);
                    float distance = MathF.Sqrt(result.DistanceSquared);
                    if (distance > Core.SpeculativeDistance + radius)
                    {
                        return manifold;
                    }

                    float invDistance = 1.0f / distance;
                    normal.X *= invDistance;
                    normal.Y *= invDistance;

                    Vec2 c1 = B2Math.MulAdd(v12, localPolyA.Radius, normal);
                    Vec2 c2 = B2Math.MulAdd(v22, -localPolyB.Radius, normal);

                    manifold.Normal = normal;
                    manifold.Point1.AnchorA = B2Math.Lerp(c1, c2, 0.5f);
                    manifold.Point1.Separation = distance - radius;
                    manifold.Point1.Id = MakeId(i12, i22);
                    manifold.PointCount = 1;
                }
                else
                {
                    // Edge region
                    manifold = ClipPolygons(localPolyA, localPolyB, edgeA, edgeB, flip);
                }
            }
            else
            {
                // Polygons overlap
                manifold = ClipPolygons(localPolyA, localPolyB, edgeA, edgeB, flip);
            }

            // Convert manifold to world space
            if (manifold.PointCount > 0)
            {
                manifold.Normal = B2Math.RotateVector(xfA.Q, manifold.Normal);
                for (int i = 0; i < manifold.PointCount; ++i)
                {
                    ref ManifoldPoint mp = ref manifold.Points[i];

                    // anchor points relative to shape origin in world space
                    mp.AnchorA = B2Math.RotateVector(xfA.Q, B2Math.Add(mp.AnchorA, origin));
                    mp.AnchorB = B2Math.Add(mp.AnchorA, B2Math.Sub(xfA.P, xfB.P));
                    mp.Point = B2Math.Add(xfA.P, mp.AnchorA);
                }
            }

            return manifold;
        }

        public static Manifold CollideSegmentAndCircle(in Segment segmentA, in Transform xfA, in Circle circleB, in Transform xfB)
        {
            Capsule capsuleA = new(segmentA.Point1, segmentA.Point2, 0.0f);
            return CollideCapsuleAndCircle(capsuleA, xfA, circleB, xfB);
        }

        public static Manifold CollideSegmentAndPolygon(in Segment segmentA, in Transform xfA, in Polygon polygonB, in Transform xfB)
        {
            Polygon polygonA = MakeCapsule(segmentA.Point1, segmentA.Point2, 0.0f);
            return CollidePolygons(polygonA, xfA, polygonB, xfB);
        }

        public static Manifold CollideChainSegmentAndCircle(
            in ChainSegment segmentA,
            in Transform xfA,
            in Circle circleB,
            in Transform xfB)
        {
            Manifold manifold = new();

            Transform xf = B2Math.InvMulTransforms(xfA, xfB);

            // Compute circle in frame of segment
            Vec2 pB = B2Math.TransformPoint(xf, circleB.Center);

            Vec2 p1 = segmentA.Segment.Point1;
            Vec2 p2 = segmentA.Segment.Point2;
            Vec2 e = B2Math.Sub(p2, p1);

            // Normal points to the right
            float offset = B2Math.Dot(B2Math.RightPerp(e), B2Math.Sub(pB, p1));
            if (offset < 0.0f)
            {
                // collision is one-sided
                return manifold;
            }

            // Barycentric coordinates
            float u = B2Math.Dot(e, B2Math.Sub(p2, pB));
            float v = B2Math.Dot(e, B2Math.Sub(pB, p1));

            Vec2 pA;

            if (v <= 0.0f)
            {
                // Behind point1?
                // Is pB in the Voronoi region of the previous edge?
                Vec2 prevEdge = B2Math.Sub(p1, segmentA.Ghost1);
                float uPrev = B2Math.Dot(prevEdge, B2Math.Sub(pB, p1));
                if (uPrev <= 0.0f)
                {
                    return manifold;
                }

                pA = p1;
            }
            else if (u <= 0.0f)
            {
                // Ahead of point2?
                Vec2 nextEdge = B2Math.Sub(segmentA.Ghost2, p2);
                float vNext = B2Math.Dot(nextEdge, B2Math.Sub(pB, p2));

                // Is pB in the Voronoi region of the next edge?
                if (vNext > 0.0f)
                {
                    return manifold;
                }

                pA = p2;
            }
            else
            {
                float ee = B2Math.Dot(e, e);
                pA = new(u * p1.X + v * p2.X, u * p1.Y + v * p2.Y);
                pA = ee > 0.0f ? B2Math.MulSV(1.0f / ee, pA) : p1;
            }

            var (normal, distance) = B2Math.Sub(pB, pA).GetLengthAndNormalize();

            float radius = circleB.Radius;
            float separation = distance - radius;
            if (separation > Core.SpeculativeDistance)
            {
                return manifold;
            }

            Vec2 cA = pA;
            Vec2 cB = B2Math.MulAdd(pB, -radius, normal);
            Vec2 contactPointA = B2Math.Lerp(cA, cB, 0.5f);

            manifold.Normal = B2Math.RotateVector(xfA.Q, normal);

            ref ManifoldPoint mp = ref manifold.Points[0];
            mp.AnchorA = B2Math.RotateVector(xfA.Q, contactPointA);
            mp.AnchorB = B2Math.Add(mp.AnchorA, B2Math.Sub(xfA.P, xfB.P));
            mp.Point = B2Math.Add(xfA.P, mp.AnchorA);
            mp.Separation = separation;
            mp.Id = 0;
            manifold.PointCount = 1;
            return manifold;
        }

        public static Manifold CollideChainSegmentAndCapsule(
            in ChainSegment segmentA,
            in Transform xfA,
            in Capsule capsuleB,
            in Transform xfB,
            ref DistanceCache cache)
        {
            Polygon polyB = MakeCapsule(capsuleB.Center1, capsuleB.Center2, capsuleB.Radius);
            return CollideChainSegmentAndPolygon(segmentA, xfA, polyB, xfB, ref cache);
        }

        private static Manifold ClipSegments(
            Vec2 a1,
            Vec2 a2,
            Vec2 b1,
            Vec2 b2,
            Vec2 normal,
            float ra,
            float rb,
            ushort id1,
            ushort id2)
        {
            Manifold manifold = new();

            Vec2 tangent = B2Math.LeftPerp(normal);

            // Barycentric coordinates of each point relative to a1 along tangent
            float lower1 = 0.0f;
            float upper1 = B2Math.Dot(B2Math.Sub(a2, a1), tangent);

            // Incident edge points opposite of tangent due to CCW winding
            float upper2 = B2Math.Dot(B2Math.Sub(b1, a1), tangent);
            float lower2 = B2Math.Dot(B2Math.Sub(b2, a1), tangent);

            // Do segments overlap?
            if (upper2 < lower1 || upper1 < lower2)
            {
                return manifold;
            }

            Vec2 vLower;
            if (lower2 < lower1 && upper2 - lower2 > float.Epsilon)
            {
                vLower = B2Math.Lerp(b2, b1, (lower1 - lower2) / (upper2 - lower2));
            }
            else
            {
                vLower = b2;
            }

            Vec2 vUpper;
            if (upper2 > upper1 && upper2 - lower2 > float.Epsilon)
            {
                vUpper = B2Math.Lerp(b2, b1, (upper1 - lower2) / (upper2 - lower2));
            }
            else
            {
                vUpper = b1;
            }

            // todo vLower can be very close to vUpper, reduce to one point?

            float separationLower = B2Math.Dot(B2Math.Sub(vLower, a1), normal);
            float separationUpper = B2Math.Dot(B2Math.Sub(vUpper, a1), normal);

            // Put contact points at midpoint, accounting for radii
            vLower = B2Math.MulAdd(vLower, 0.5f * (ra - rb - separationLower), normal);
            vUpper = B2Math.MulAdd(vUpper, 0.5f * (ra - rb - separationUpper), normal);

            float radius = ra + rb;

            manifold.Normal = normal;
            {
                ref ManifoldPoint cp = ref manifold.Points[0];
                cp.AnchorA = vLower;
                cp.Separation = separationLower - radius;
                cp.Id = id1;
            }

            {
                ref ManifoldPoint cp = ref manifold.Points[1];
                cp.AnchorA = vUpper;
                cp.Separation = separationUpper - radius;
                cp.Id = id2;
            }

            manifold.PointCount = 2;

            return manifold;
        }

        private enum NormalType
        {
            // This means the normal points in a direction that is non-smooth relative to a convex vertex and should be skipped
            NormalSkip,

            // This means the normal points in a direction that is smooth relative to a convex vertex and should be used for collision
            NormalAdmit,

            // This means the normal is in a region of a concave vertex and should be snapped to the segment normal
            NormalSnap
        };

        public struct ChainSegmentParams
        {
            public Vec2 Edge1;

            public Vec2 Normal0;

            public Vec2 Normal2;

            public bool Convex1;

            public bool Convex2;
        };

        // Evaluate Gauss map
        // See https://box2d.org/posts/2020/06/ghost-collisions/
        private static NormalType ClassifyNormal(ChainSegmentParams @params, Vec2 normal)
        {
            const float SinTol = 0.01f;

            if (B2Math.Dot(normal, @params.Edge1) <= 0.0f)
            {
                // Normal points towards the segment tail
                if (@params.Convex1)
                {
                    if (B2Math.Cross(normal, @params.Normal0) > SinTol)
                    {
                        return NormalType.NormalSkip;
                    }

                    return NormalType.NormalAdmit;
                }
                else
                {
                    return NormalType.NormalSnap;
                }
            }
            else
            {
                // Normal points towards segment head
                if (@params.Convex2)
                {
                    if (B2Math.Cross(@params.Normal2, normal) > SinTol)
                    {
                        return NormalType.NormalSkip;
                    }

                    return NormalType.NormalAdmit;
                }
                else
                {
                    return NormalType.NormalSnap;
                }
            }
        }

        public static Manifold CollideChainSegmentAndPolygon(
            in ChainSegment segmentA,
            in Transform xfA,
            in Polygon polygonB,
            in Transform xfB,
            ref DistanceCache cache)
        {
            Manifold manifold = new();

            Transform xf = B2Math.InvMulTransforms(xfA, xfB);

            Vec2 centroidB = B2Math.TransformPoint(xf, polygonB.Centroid);
            float radiusB = polygonB.Radius;

            Vec2 p1 = segmentA.Segment.Point1;
            Vec2 p2 = segmentA.Segment.Point2;

            Vec2 edge1 = B2Math.Sub(p2, p1).Normalize;

            ChainSegmentParams smoothParams = new()
            {
                Edge1 = edge1
            };

            const float ConvexTol = 0.01f;
            Vec2 edge0 = B2Math.Sub(p1, segmentA.Ghost1).Normalize;
            smoothParams.Normal0 = B2Math.RightPerp(edge0);
            smoothParams.Convex1 = B2Math.Cross(edge0, edge1) >= ConvexTol;

            Vec2 edge2 = B2Math.Sub(segmentA.Ghost2, p2).Normalize;
            smoothParams.Normal2 = B2Math.RightPerp(edge2);
            smoothParams.Convex2 = B2Math.Cross(edge1, edge2) >= ConvexTol;

            // Normal points to the right
            Vec2 normal1 = B2Math.RightPerp(edge1);
            bool behind1 = B2Math.Dot(normal1, B2Math.Sub(centroidB, p1)) < 0.0f;
            bool behind0 = true;
            bool behind2 = true;
            if (smoothParams.Convex1)
            {
                behind0 = B2Math.Dot(smoothParams.Normal0, B2Math.Sub(centroidB, p1)) < 0.0f;
            }

            if (smoothParams.Convex2)
            {
                behind2 = B2Math.Dot(smoothParams.Normal2, B2Math.Sub(centroidB, p2)) < 0.0f;
            }

            if (behind1 && behind0 && behind2)
            {
                // one-sided collision
                return manifold;
            }

            // Get polygonB in frameA
            int count = polygonB.Count;
            FixedArray8<Vec2> vertices = new();
            FixedArray8<Vec2> normals = new();
            for (int i = 0; i < count; ++i)
            {
                vertices[i] = B2Math.TransformPoint(xf, polygonB.Vertices[i]);
                normals[i] = B2Math.RotateVector(xf.Q, polygonB.Normals[i]);
            }

            // Distance doesn't work correctly with partial polygons
            DistanceInput input;
            input.ProxyA = DistanceFunc.MakeProxy(stackalloc Vec2[2] { segmentA.Segment.Point1, segmentA.Segment.Point2 }, 2, 0.0f);
            input.ProxyB = DistanceFunc.MakeProxy(vertices.Span, count, 0.0f);
            input.TransformA = Transform.Identity;
            input.TransformB = Transform.Identity;
            input.UseRadii = false;

            DistanceOutput output = DistanceFunc.ShapeDistance(ref cache, input, null, 0);

            if (output.Distance > radiusB + Core.SpeculativeDistance)
            {
                return manifold;
            }

            // Snap concave normals for partial polygon
            Vec2 n0 = smoothParams.Convex1 ? smoothParams.Normal0 : normal1;
            Vec2 n2 = smoothParams.Convex2 ? smoothParams.Normal2 : normal1;

            // Index of incident vertex on polygon
            int incidentIndex = -1;
            int incidentNormal = -1;

            if (behind1 == false && output.Distance > 0.1f * Core.LinearSlop)
            {
                // The closest features may be two vertices or an edge and a vertex even when there should
                // be face contact

                if (cache.Count == 1)
                {
                    // vertex-vertex collision
                    Vec2 pA = output.PointA;
                    Vec2 pB = output.PointB;

                    Vec2 normal = B2Math.Sub(pB, pA).Normalize;

                    NormalType type = ClassifyNormal(smoothParams, normal);
                    if (type == NormalType.NormalSkip)
                    {
                        return manifold;
                    }

                    if (type == NormalType.NormalAdmit)
                    {
                        manifold.Normal = B2Math.RotateVector(xfA.Q, normal);
                        ref ManifoldPoint cp = ref manifold.Point1;
                        cp.AnchorA = B2Math.RotateVector(xfA.Q, pA);
                        cp.AnchorB = B2Math.Add(cp.AnchorA, B2Math.Sub(xfA.P, xfB.P));
                        cp.Point = B2Math.Add(xfA.P, cp.AnchorA);
                        cp.Separation = output.Distance - radiusB;
                        cp.Id = MakeId(cache.IndexA[0], cache.IndexB[0]);
                        manifold.PointCount = 1;
                        return manifold;
                    }

                    // fall through b2_normalSnap
                    incidentIndex = cache.IndexB[0];
                }
                else
                {
                    // vertex-edge collision
                    Debug.Assert(cache.Count == 2);

                    int ia1 = cache.IndexA[0];
                    int ia2 = cache.IndexA[1];
                    int ib1 = cache.IndexB[0];
                    int ib2 = cache.IndexB[1];

                    if (ia1 == ia2)
                    {
                        // 1 point on A, expect 2 points on B
                        Debug.Assert(ib1 != ib2);

                        // Find polygon normal most aligned with vector between closest points.
                        // This effectively sorts ib1 and ib2
                        Vec2 normalB = B2Math.Sub(output.PointA, output.PointB);
                        float dot1 = B2Math.Dot(normalB, normals[ib1]);
                        float dot2 = B2Math.Dot(normalB, normals[ib2]);
                        int ib = dot1 > dot2 ? ib1 : ib2;

                        // Use accurate normal
                        normalB = normals[ib];

                        NormalType type = ClassifyNormal(smoothParams, B2Math.Neg(normalB));
                        if (type == NormalType.NormalSkip)
                        {
                            return manifold;
                        }

                        if (type == NormalType.NormalAdmit)
                        {
                            // Get polygon edge associated with normal
                            ib1 = ib;
                            ib2 = ib < count - 1 ? ib + 1 : 0;

                            Vec2 b1 = vertices[ib1];
                            Vec2 b2 = vertices[ib2];

                            // Find incident segment vertex
                            dot1 = B2Math.Dot(normalB, B2Math.Sub(p1, b1));
                            dot2 = B2Math.Dot(normalB, B2Math.Sub(p2, b1));

                            if (dot1 < dot2)
                            {
                                if (B2Math.Dot(n0, normalB) < B2Math.Dot(normal1, normalB))
                                {
                                    // Neighbor is incident
                                    return manifold;
                                }
                            }
                            else
                            {
                                if (B2Math.Dot(n2, normalB) < B2Math.Dot(normal1, normalB))
                                {
                                    // Neighbor is incident
                                    return manifold;
                                }
                            }

                            {
                                manifold = ClipSegments(b1, b2, p1, p2, normalB, radiusB, 0.0f, MakeId(ib1, 1), MakeId(ib2, 0));
                                manifold.Normal = B2Math.RotateVector(xfA.Q, B2Math.Neg(normalB));
                                manifold.Point1.AnchorA = B2Math.RotateVector(xfA.Q, manifold.Point1.AnchorA);
                                manifold.Points[1].AnchorA = B2Math.RotateVector(xfA.Q, manifold.Points[1].AnchorA);
                                Vec2 pAB = B2Math.Sub(xfA.P, xfB.P);
                                manifold.Point1.AnchorB = B2Math.Add(manifold.Point1.AnchorA, pAB);
                                manifold.Points[1].AnchorB = B2Math.Add(manifold.Points[1].AnchorA, pAB);
                                manifold.Point1.Point = B2Math.Add(xfA.P, manifold.Point1.AnchorA);
                                manifold.Points[1].Point = B2Math.Add(xfA.P, manifold.Points[1].AnchorA);
                                return manifold;
                            }
                        }

                        // fall through b2_normalSnap
                        incidentNormal = ib;
                    }
                    else
                    {
                        // Get index of incident polygonB vertex
                        float dot1 = B2Math.Dot(normal1, B2Math.Sub(vertices[ib1], p1));
                        float dot2 = B2Math.Dot(normal1, B2Math.Sub(vertices[ib2], p2));
                        incidentIndex = dot1 < dot2 ? ib1 : ib2;
                    }
                }
            }
            else
            {
                // SAT edge normal
                float edgeSeparation = float.MaxValue;

                for (int i = 0; i < count; ++i)
                {
                    float s = B2Math.Dot(normal1, B2Math.Sub(vertices[i], p1));
                    if (s < edgeSeparation)
                    {
                        edgeSeparation = s;
                        incidentIndex = i;
                    }
                }

                // Check convex neighbor for edge separation
                if (smoothParams.Convex1)
                {
                    float s0 = float.MaxValue;

                    for (int i = 0; i < count; ++i)
                    {
                        float s = B2Math.Dot(smoothParams.Normal0, B2Math.Sub(vertices[i], p1));
                        if (s < s0)
                        {
                            s0 = s;
                        }
                    }

                    if (s0 > edgeSeparation)
                    {
                        edgeSeparation = s0;

                        // Indicate neighbor owns edge separation
                        incidentIndex = -1;
                    }
                }

                // Check convex neighbor for edge separation
                if (smoothParams.Convex2)
                {
                    float s2 = float.MaxValue;

                    for (int i = 0; i < count; ++i)
                    {
                        float s = B2Math.Dot(smoothParams.Normal2, B2Math.Sub(vertices[i], p2));
                        if (s < s2)
                        {
                            s2 = s;
                        }
                    }

                    if (s2 > edgeSeparation)
                    {
                        edgeSeparation = s2;

                        // Indicate neighbor owns edge separation
                        incidentIndex = -1;
                    }
                }

                // SAT polygon normals
                float polygonSeparation = -float.MaxValue;
                int referenceIndex = -1;

                for (int i = 0; i < count; ++i)
                {
                    Vec2 n = normals[i];

                    NormalType type = ClassifyNormal(smoothParams, B2Math.Neg(n));
                    if (type != NormalType.NormalAdmit)
                    {
                        continue;
                    }

                    // Check the infinite sides of the partial polygon
                    // if ((smoothParams.convex1 && b2Cross(n0, n) > 0.0f) || (smoothParams.convex2 && b2Cross(n, n2) > 0.0f))
                    //{
                    //	continue;
                    //}

                    Vec2 p = vertices[i];
                    float s = Math.Min(B2Math.Dot(n, B2Math.Sub(p2, p)), B2Math.Dot(n, B2Math.Sub(p1, p)));

                    if (s > polygonSeparation)
                    {
                        polygonSeparation = s;
                        referenceIndex = i;
                    }
                }

                if (polygonSeparation > edgeSeparation)
                {
                    int ia1 = referenceIndex;
                    int ia2 = ia1 < count - 1 ? ia1 + 1 : 0;
                    Vec2 a1 = vertices[ia1];
                    Vec2 a2 = vertices[ia2];

                    Vec2 n = normals[ia1];

                    float dot1 = B2Math.Dot(n, B2Math.Sub(p1, a1));
                    float dot2 = B2Math.Dot(n, B2Math.Sub(p2, a1));

                    if (dot1 < dot2)
                    {
                        if (B2Math.Dot(n0, n) < B2Math.Dot(normal1, n))
                        {
                            // Neighbor is incident
                            return manifold;
                        }
                    }
                    else
                    {
                        if (B2Math.Dot(n2, n) < B2Math.Dot(normal1, n))
                        {
                            // Neighbor is incident
                            return manifold;
                        }
                    }

                    manifold = ClipSegments(a1, a2, p1, p2, normals[ia1], radiusB, 0.0f, MakeId(ia1, 1), MakeId(ia2, 0));
                    manifold.Normal = B2Math.RotateVector(xfA.Q, B2Math.Neg(normals[ia1]));
                    manifold.Point1.AnchorA = B2Math.RotateVector(xfA.Q, manifold.Point1.AnchorA);
                    manifold.Points[1].AnchorA = B2Math.RotateVector(xfA.Q, manifold.Points[1].AnchorA);
                    Vec2 pAB = B2Math.Sub(xfA.P, xfB.P);
                    manifold.Point1.AnchorB = B2Math.Add(manifold.Point1.AnchorA, pAB);
                    manifold.Points[1].AnchorB = B2Math.Add(manifold.Points[1].AnchorA, pAB);
                    manifold.Point1.Point = B2Math.Add(xfA.P, manifold.Point1.AnchorA);
                    manifold.Points[1].Point = B2Math.Add(xfA.P, manifold.Points[1].AnchorA);
                    return manifold;
                }

                if (incidentIndex == -1)
                {
                    // neighboring segment is the separating axis
                    return manifold;
                }

                // fall through segment normal axis
            }

            Debug.Assert(incidentNormal != -1 || incidentIndex != -1);

            {
                // Segment normal

                // Find incident polygon normal: normal adjacent to deepest vertex that is most anti-parallel to segment normal
                Vec2 b1, b2;
                int ib1, ib2;

                if (incidentNormal != -1)
                {
                    ib1 = incidentNormal;
                    ib2 = ib1 < count - 1 ? ib1 + 1 : 0;
                    b1 = vertices[ib1];
                    b2 = vertices[ib2];
                }
                else
                {
                    int i2 = incidentIndex;
                    int i1 = i2 > 0 ? i2 - 1 : count - 1;
                    float d1 = B2Math.Dot(normal1, normals[i1]);
                    float d2 = B2Math.Dot(normal1, normals[i2]);
                    if (d1 < d2)
                    {
                        ib1 = i1;
                        ib2 = i2;
                        b1 = vertices[ib1];
                        b2 = vertices[ib2];
                    }
                    else
                    {
                        ib1 = i2;
                        ib2 = i2 < count - 1 ? i2 + 1 : 0;
                        b1 = vertices[ib1];
                        b2 = vertices[ib2];
                    }
                }

                manifold = ClipSegments(p1, p2, b1, b2, normal1, 0.0f, radiusB, MakeId(0, ib2), MakeId(1, ib1));
                manifold.Normal = B2Math.RotateVector(xfA.Q, manifold.Normal);
                manifold.Point1.AnchorA = B2Math.RotateVector(xfA.Q, manifold.Point1.AnchorA);
                manifold.Points[1].AnchorA = B2Math.RotateVector(xfA.Q, manifold.Points[1].AnchorA);
                Vec2 pAB = B2Math.Sub(xfA.P, xfB.P);
                manifold.Point1.AnchorB = B2Math.Add(manifold.Point1.AnchorA, pAB);
                manifold.Points[1].AnchorB = B2Math.Add(manifold.Points[1].AnchorA, pAB);
                manifold.Point1.Point = B2Math.Add(xfA.P, manifold.Point1.AnchorA);
                manifold.Points[1].Point = B2Math.Add(xfA.P, manifold.Points[1].AnchorA);

                return manifold;
            }
        }
    }
}