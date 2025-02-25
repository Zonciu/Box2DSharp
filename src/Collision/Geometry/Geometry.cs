using System;
using System.Diagnostics;

namespace Box2DSharp
{
    public static class Geometry
    {
        public static bool IsValidRay(RayCastInput input)
        {
            var isValid = B2Math.Vec2_IsValid(input.Origin) && B2Math.Vec2_IsValid(input.Translation) && B2Math.IsValid(input.MaxFraction) && 0.0f <= input.MaxFraction && input.MaxFraction < Core.Huge;
            return isValid;
        }

        public static Vec2 ComputePolygonCentroid(Span<Vec2> vertices, int count)
        {
            Vec2 center = new(0.0f, 0.0f);
            float area = 0.0f;

            // Get a reference point for forming triangles.
            // Use the first vertex to reduce round-off errors.
            Vec2 origin = vertices[0];

            const float inv3 = 1.0f / 3.0f;

            for (int i = 1; i < count - 1; ++i)
            {
                // Triangle edges
                Vec2 e1 = B2Math.Sub(vertices[i], origin);
                Vec2 e2 = B2Math.Sub(vertices[i + 1], origin);
                float a = 0.5f * B2Math.Cross(e1, e2);

                // Area weighted centroid
                center = B2Math.MulAdd(center, a * inv3, B2Math.Add(e1, e2));
                area += a;
            }

            Debug.Assert(area > float.Epsilon);
            float invArea = 1.0f / area;
            center.X *= invArea;
            center.Y *= invArea;

            // Restore offset
            center = B2Math.Add(origin, center);

            return center;
        }

        public static Polygon MakePolygon(Hull hull, float radius)
        {
            Debug.Assert(HullFunc.ValidateHull(hull));

            if (hull.Count < 3)
            {
                // Handle a bad hull when assertions are disabled
                return MakeSquare(0.5f);
            }

            Polygon shape = new();
            shape.Count = hull.Count;
            shape.Radius = radius;

            // Copy vertices
            for (int i = 0; i < shape.Count; ++i)
            {
                shape.Vertices[i] = hull.Points[i];
            }

            // Compute normals. Ensure the edges have non-zero length.
            for (int i = 0; i < shape.Count; ++i)
            {
                int i1 = i;
                int i2 = i + 1 < shape.Count ? i + 1 : 0;
                Vec2 edge = B2Math.Sub(shape.Vertices[i2], shape.Vertices[i1]);
                Debug.Assert(B2Math.Dot(edge, edge) > float.Epsilon * float.Epsilon);
                shape.Normals[i] = B2Math.CrossVS(edge, 1.0f).Normalize;
            }

            shape.Centroid = ComputePolygonCentroid(shape.Vertices, shape.Count);

            return shape;
        }

        public static Polygon MakeOffsetPolygon(Hull hull, float radius, Transform transform)
        {
            Debug.Assert(HullFunc.ValidateHull(hull));

            if (hull.Count < 3)
            {
                // Handle a bad hull when assertions are disabled
                return MakeSquare(0.5f);
            }

            Polygon shape = new();
            shape.Count = hull.Count;
            shape.Radius = radius;

            // Copy vertices
            for (int i = 0; i < shape.Count; ++i)
            {
                shape.Vertices[i] = B2Math.TransformPoint(transform, hull.Points[i]);
            }

            // Compute normals. Ensure the edges have non-zero length.
            for (int i = 0; i < shape.Count; ++i)
            {
                int i1 = i;
                int i2 = i + 1 < shape.Count ? i + 1 : 0;
                Vec2 edge = B2Math.Sub(shape.Vertices[i2], shape.Vertices[i1]);
                Debug.Assert(B2Math.Dot(edge, edge) > float.Epsilon * float.Epsilon);
                shape.Normals[i] = B2Math.CrossVS(edge, 1.0f).Normalize;
            }

            shape.Centroid = ComputePolygonCentroid(shape.Vertices, shape.Count);

            return shape;
        }

        /// <summary>
        /// 创建一个正方形
        /// </summary>
        /// <param name="h"></param>
        /// <returns></returns>
        public static Polygon MakeSquare(float h)
        {
            return MakeBox(h, h);
        }

        /// <summary>
        /// 创建一个矩形
        /// </summary>
        /// <param name="hx"></param>
        /// <param name="hy"></param>
        /// <returns></returns>
        public static Polygon MakeBox(float hx, float hy)
        {
            Debug.Assert(B2Math.IsValid(hx) && hx > 0.0f);
            Debug.Assert(B2Math.IsValid(hy) && hy > 0.0f);

            Polygon shape = new();
            shape.Count = 4;
            shape.Vertices[0] = new Vec2(-hx, -hy);
            shape.Vertices[1] = new Vec2(hx, -hy);
            shape.Vertices[2] = new Vec2(hx, hy);
            shape.Vertices[3] = new Vec2(-hx, hy);
            shape.Normals[0] = new Vec2(0.0f, -1.0f);
            shape.Normals[1] = new Vec2(1.0f, 0.0f);
            shape.Normals[2] = new Vec2(0.0f, 1.0f);
            shape.Normals[3] = new Vec2(-1.0f, 0.0f);
            shape.Radius = 0.0f;
            shape.Centroid = Vec2.Zero;
            return shape;
        }

        /// <summary>
        /// 创建一个圆角矩形
        /// </summary>
        /// <param name="hx"></param>
        /// <param name="hy"></param>
        /// <param name="radius"></param>
        /// <returns></returns>
        public static Polygon MakeRoundedBox(float hx, float hy, float radius)
        {
            Polygon shape = MakeBox(hx, hy);
            shape.Radius = radius;
            return shape;
        }

        /// <summary>
        /// 创建一个偏移矩形
        /// </summary>
        /// <param name="hx"></param>
        /// <param name="hy"></param>
        /// <param name="center"></param>
        /// <param name="rotation"></param>
        /// <returns></returns>
        public static Polygon MakeOffsetBox(float hx, float hy, Vec2 center, Rot rotation)
        {
            Transform xf;
            xf.P = center;
            xf.Q = rotation;

            Polygon shape = new();
            shape.Count = 4;
            shape.Vertices[0] = B2Math.TransformPoint(xf, new Vec2(-hx, -hy));
            shape.Vertices[1] = B2Math.TransformPoint(xf, new Vec2(hx, -hy));
            shape.Vertices[2] = B2Math.TransformPoint(xf, new Vec2(hx, hy));
            shape.Vertices[3] = B2Math.TransformPoint(xf, new Vec2(-hx, hy));
            shape.Normals[0] = B2Math.RotateVector(xf.Q, new Vec2(0.0f, -1.0f));
            shape.Normals[1] = B2Math.RotateVector(xf.Q, new Vec2(1.0f, 0.0f));
            shape.Normals[2] = B2Math.RotateVector(xf.Q, new Vec2(0.0f, 1.0f));
            shape.Normals[3] = B2Math.RotateVector(xf.Q, new Vec2(-1.0f, 0.0f));
            shape.Radius = 0.0f;
            shape.Centroid = center;
            return shape;
        }

        /// <summary>
        /// 对多边形应用位移
        /// </summary>
        /// <param name="transform"></param>
        /// <param name="polygon"></param>
        /// <returns></returns>
        public static Polygon TransformPolygon(Transform transform, Polygon polygon)
        {
            Polygon p = polygon;

            for (int i = 0; i < p.Count; ++i)
            {
                p.Vertices[i] = B2Math.TransformPoint(transform, p.Vertices[i]);
                p.Normals[i] = B2Math.RotateVector(transform.Q, p.Normals[i]);
            }

            p.Centroid = B2Math.TransformPoint(transform, p.Centroid);

            return p;
        }

        /// <summary>
        /// 计算圆的质量
        /// </summary>
        /// <param name="shape"></param>
        /// <param name="density"></param>
        /// <returns></returns>
        public static MassData ComputeCircleMass(Circle shape, float density)
        {
            float rr = shape.Radius * shape.Radius;

            MassData massData;
            massData.Mass = density * B2Math.Pi * rr;
            massData.Center = shape.Center;

            // inertia about the local origin
            massData.RotationalInertia = massData.Mass * (0.5f * rr + B2Math.Dot(shape.Center, shape.Center));

            return massData;
        }

        /// <summary>
        /// 计算胶囊体质量
        /// </summary>
        /// <param name="shape"></param>
        /// <param name="density"></param>
        /// <returns></returns>
        public static MassData ComputeCapsuleMass(Capsule shape, float density)
        {
            float radius = shape.Radius;
            float rr = radius * radius;
            Vec2 p1 = shape.Center1;
            Vec2 p2 = shape.Center2;
            float length = B2Math.Sub(p2, p1).Length;
            float ll = length * length;

            float circleMass = density * (B2Math.Pi * radius * radius);
            float boxMass = density * (2.0f * radius * length);

            MassData massData;
            massData.Mass = circleMass + boxMass;
            massData.Center.X = 0.5f * (p1.X + p2.X);
            massData.Center.Y = 0.5f * (p1.Y + p2.Y);

            // two offset half circles, both halves add up to full circle and each half is offset by half length
            // semi-circle centroid = 4 r / 3 pi
            // Need to apply parallel-axis theorem twice:
            // 1. shift semi-circle centroid to origin
            // 2. shift semi-circle to box end
            // m * ((h + lc)^2 - lc^2) = m * (h^2 + 2 * h * lc)
            // See: https://en.wikipedia.org/wiki/Parallel_axis_theorem
            // I verified this formula by computing the convex hull of a 128 vertex capsule

            // half circle centroid
            float lc = 4.0f * radius / (3.0f * B2Math.Pi);

            // half length of rectangular portion of capsule
            float h = 0.5f * length;

            float circleInertia = circleMass * (0.5f * rr + h * h + 2.0f * h * lc);
            float boxInertia = boxMass * (4.0f * rr + ll) / 12.0f;
            massData.RotationalInertia = circleInertia + boxInertia;

            // inertia about the local origin
            massData.RotationalInertia += massData.Mass * B2Math.Dot(massData.Center, massData.Center);

            return massData;
        }

        /// <summary>
        /// 计算多边形质量
        /// </summary>
        /// <param name="shape"></param>
        /// <param name="density"></param>
        /// <returns></returns>
        public static MassData ComputePolygonMass(Polygon shape, float density)
        {
            // Polygon mass, centroid, and inertia.
            // Let rho be the polygon density in mass per unit area.
            // Then:
            // mass = rho * int(dA)
            // centroid.x = (1/mass) * rho * int(x * dA)
            // centroid.y = (1/mass) * rho * int(y * dA)
            // I = rho * int((x*x + y*y) * dA)
            //
            // We can compute these integrals by summing all the integrals
            // for each triangle of the polygon. To evaluate the integral
            // for a single triangle, we make a change of variables to
            // the (u,v) coordinates of the triangle:
            // x = x0 + e1x * u + e2x * v
            // y = y0 + e1y * u + e2y * v
            // where 0 <= u && 0 <= v && u + v <= 1.
            //
            // We integrate u from [0,1-v] and then v from [0,1].
            // We also need to use the Jacobian of the transformation:
            // D = cross(e1, e2)
            //
            // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
            //
            // The rest of the derivation is handled by computer algebra.

            Debug.Assert(shape.Count > 0);

            if (shape.Count == 1)
            {
                Circle circle;
                circle.Center = shape.Vertices[0];
                circle.Radius = shape.Radius;
                return ComputeCircleMass(circle, density);
            }

            if (shape.Count == 2)
            {
                Capsule capsule = new()
                {
                    Center1 = shape.Vertices[0],
                    Center2 = shape.Vertices[1],
                    Radius = shape.Radius
                };
                return ComputeCapsuleMass(capsule, density);
            }

            Span<Vec2> vertices = stackalloc Vec2[Core.MaxPolygonVertices];
            int count = shape.Count;
            float radius = shape.Radius;

            if (radius > 0.0f)
            {
                // Approximate mass of rounded polygons by pushing out the vertices.
                float sqrt2 = 1.412f;
                for (int i = 0; i < count; ++i)
                {
                    int j = i == 0 ? count - 1 : i - 1;
                    Vec2 n1 = shape.Normals[j];
                    Vec2 n2 = shape.Normals[i];

                    Vec2 mid = B2Math.Add(n1, n2).Normalize;
                    vertices[i] = B2Math.MulAdd(shape.Vertices[i], sqrt2 * radius, mid);
                }
            }
            else
            {
                for (int i = 0; i < count; ++i)
                {
                    vertices[i] = shape.Vertices[i];
                }
            }

            Vec2 center = new(0.0f, 0.0f);
            float area = 0.0f;
            float rotationalInertia = 0.0f;

            // Get a reference point for forming triangles.
            // Use the first vertex to reduce round-off errors.
            Vec2 r = vertices[0];

            const float inv3 = 1.0f / 3.0f;

            for (int i = 1; i < count - 1; ++i)
            {
                // Triangle edges
                Vec2 e1 = B2Math.Sub(vertices[i], r);
                Vec2 e2 = B2Math.Sub(vertices[i + 1], r);

                float D = B2Math.Cross(e1, e2);

                float triangleArea = 0.5f * D;
                area += triangleArea;

                // Area weighted centroid, r at origin
                center = B2Math.MulAdd(center, triangleArea * inv3, B2Math.Add(e1, e2));

                float ex1 = e1.X, ey1 = e1.Y;
                float ex2 = e2.X, ey2 = e2.Y;

                float intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
                float inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

                rotationalInertia += (0.25f * inv3 * D) * (intx2 + inty2);
            }

            MassData massData;

            // Total mass
            massData.Mass = density * area;

            // Center of mass, shift back from origin at r
            Debug.Assert(area > float.Epsilon);
            float invArea = 1.0f / area;
            center.X *= invArea;
            center.Y *= invArea;
            massData.Center = B2Math.Add(r, center);

            // Inertia tensor relative to the local origin (point s).
            massData.RotationalInertia = density * rotationalInertia;

            // Shift to center of mass then to original body origin.
            massData.RotationalInertia += massData.Mass * (B2Math.Dot(massData.Center, massData.Center) - B2Math.Dot(center, center));

            return massData;
        }

        /// <summary>
        /// 计算圆的包围盒
        /// </summary>
        /// <param name="shape"></param>
        /// <param name="xf"></param>
        /// <returns></returns>
        public static AABB ComputeCircleAABB(Circle shape, Transform xf)
        {
            Vec2 p = B2Math.TransformPoint(xf, shape.Center);
            float r = shape.Radius;

            AABB aabb = new(new(p.X - r, p.Y - r), new(p.X + r, p.Y + r));
            return aabb;
        }

        /// <summary>
        /// 计算胶囊体包围盒
        /// </summary>
        /// <param name="shape"></param>
        /// <param name="xf"></param>
        /// <returns></returns>
        public static AABB ComputeCapsuleAABB(Capsule shape, Transform xf)
        {
            Vec2 v1 = B2Math.TransformPoint(xf, shape.Center1);
            Vec2 v2 = B2Math.TransformPoint(xf, shape.Center2);

            Vec2 r = new(shape.Radius, shape.Radius);
            Vec2 lower = B2Math.Sub(B2Math.Min(v1, v2), r);
            Vec2 upper = B2Math.Add(B2Math.Max(v1, v2), r);

            AABB aabb = new(lower, upper);
            return aabb;
        }

        /// <summary>
        /// 计算多边形包围盒
        /// </summary>
        /// <param name="shape"></param>
        /// <param name="xf"></param>
        /// <returns></returns>
        public static AABB ComputePolygonAABB(Polygon shape, Transform xf)
        {
            Debug.Assert(shape.Count > 0);
            Vec2 lower = B2Math.TransformPoint(xf, shape.Vertices[0]);
            Vec2 upper = lower;

            for (int i = 1; i < shape.Count; ++i)
            {
                Vec2 v = B2Math.TransformPoint(xf, shape.Vertices[i]);
                lower = B2Math.Min(lower, v);
                upper = B2Math.Max(upper, v);
            }

            Vec2 r = new(shape.Radius, shape.Radius);
            lower = B2Math.Sub(lower, r);
            upper = B2Math.Add(upper, r);

            AABB aabb = new(lower, upper);
            return aabb;
        }

        /// <summary>
        /// 计算线段包围盒
        /// </summary>
        /// <param name="shape"></param>
        /// <param name="xf"></param>
        /// <returns></returns>
        public static AABB ComputeSegmentAABB(Segment shape, Transform xf)
        {
            Vec2 v1 = B2Math.TransformPoint(xf, shape.Point1);
            Vec2 v2 = B2Math.TransformPoint(xf, shape.Point2);

            Vec2 lower = B2Math.Min(v1, v2);
            Vec2 upper = B2Math.Max(v1, v2);

            AABB aabb = new(lower, upper);
            return aabb;
        }

        /// <summary>
        /// 判断给定点是否在圆内
        /// </summary>
        /// <param name="point"></param>
        /// <param name="shape"></param>
        /// <returns></returns>
        public static bool PointInCircle(Vec2 point, Circle shape)
        {
            Vec2 center = shape.Center;
            return B2Math.DistanceSquared(point, center) <= shape.Radius * shape.Radius;
        }

        /// <summary>
        /// 判断给定点是否在胶囊内
        /// </summary>
        /// <param name="point"></param>
        /// <param name="shape"></param>
        /// <returns></returns>
        public static bool PointInCapsule(Vec2 point, Capsule shape)
        {
            float rr = shape.Radius * shape.Radius;
            Vec2 p1 = shape.Center1;
            Vec2 p2 = shape.Center2;

            Vec2 d = B2Math.Sub(p2, p1);
            float dd = B2Math.Dot(d, d);
            if (dd == 0.0f)
            {
                // Capsule is really a circle
                return B2Math.DistanceSquared(point, p1) <= rr;
            }

            // Get closest point on capsule segment
            // c = p1 + t * d
            // dot(point - c, d) = 0
            // dot(point - p1 - t * d, d) = 0
            // t = dot(point - p1, d) / dot(d, d)
            float t = B2Math.Dot(B2Math.Sub(point, p1), d) / dd;
            t = Math.Clamp(t, 0.0f, 1.0f);
            Vec2 c = B2Math.MulAdd(p1, t, d);

            // Is query point within radius around closest point?
            return B2Math.DistanceSquared(point, c) <= rr;
        }

        /// <summary>
        /// 判断给定点是否在多边形内
        /// </summary>
        /// <param name="point"></param>
        /// <param name="shape"></param>
        /// <returns></returns>
        public static bool PointInPolygon(Vec2 point, Polygon shape)
        {
            DistanceInput input = new();
            input.ProxyA = DistanceFunc.MakeProxy(shape.Vertices, shape.Count, 0.0f);
            input.ProxyB = DistanceFunc.MakeProxy(point, 0.0f);
            input.TransformA = Transform.Identity;
            input.TransformB = Transform.Identity;
            input.UseRadii = false;

            DistanceCache cache = new();
            DistanceOutput output = DistanceFunc.ShapeDistance(ref cache, input, null, 0);

            return output.Distance <= shape.Radius;
        }

        /// <summary>
        /// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019  http://www.codercorner.com/blog/?p=321<br/>
        /// 对圆形发出射线
        /// </summary>
        /// <param name="input"></param>
        /// <param name="shape"></param>
        /// <returns></returns>
        public static CastOutput RayCastCircle(RayCastInput input, Circle shape)

        {
            Debug.Assert(IsValidRay(input));

            Vec2 p = shape.Center;

            CastOutput output = new();

            // Shift ray so circle center is the origin
            Vec2 s = B2Math.Sub(input.Origin, p);

            var (d, length) = input.Translation.GetLengthAndNormalize();
            if (length == 0.0f)
            {
                // zero length ray
                return output;
            }

            // Find closest point on ray to origin

            // solve: dot(s + t * d, d) = 0
            float t = -B2Math.Dot(s, d);

            // c is the closest point on the line to the origin
            Vec2 c = B2Math.MulAdd(s, t, d);

            float cc = B2Math.Dot(c, c);
            float r = shape.Radius;
            float rr = r * r;

            if (cc > rr)
            {
                // closest point is outside the circle
                return output;
            }

            // Pythagorus
            float h = MathF.Sqrt(rr - cc);

            float fraction = t - h;

            if (fraction < 0.0f || input.MaxFraction * length < fraction)
            {
                // outside the range of the ray segment
                return output;
            }

            Vec2 hitPoint = B2Math.MulAdd(s, fraction, d);

            output.Fraction = fraction / length;
            output.Normal = hitPoint.Normalize;
            output.Point = B2Math.MulAdd(p, shape.Radius, output.Normal);
            output.Hit = true;

            return output;
        }

        /// <summary>
        /// 对胶囊发出射线
        /// </summary>
        /// <param name="input"></param>
        /// <param name="shape"></param>
        /// <returns></returns>
        public static CastOutput RayCastCapsule(RayCastInput input, Capsule shape)

        {
            Debug.Assert(IsValidRay(input));

            CastOutput output = new();

            Vec2 v1 = shape.Center1;
            Vec2 v2 = shape.Center2;

            Vec2 e = B2Math.Sub(v2, v1);

            var (a, capsuleLength) = e.GetLengthAndNormalize();

            if (capsuleLength < float.Epsilon)
            {
                // Capsule is really a circle
                Circle circle = (v1, shape.Radius);
                return RayCastCircle(input, circle);
            }

            Vec2 p1 = input.Origin;
            Vec2 d = input.Translation;

            // Ray from capsule start to ray start
            Vec2 q = B2Math.Sub(p1, v1);
            float qa = B2Math.Dot(q, a);

            // Vector to ray start that is perpendicular to capsule axis
            Vec2 qp = B2Math.MulAdd(q, -qa, a);

            float radius = shape.Radius;

            // Does the ray start within the infinite length capsule?
            if (B2Math.Dot(qp, qp) < radius * radius)
            {
                if (qa < 0.0f)
                {
                    // start point behind capsule segment
                    Circle circle = (v1, shape.Radius);
                    return RayCastCircle(input, circle);
                }

                if (qa > 1.0f)
                {
                    // start point ahead of capsule segment
                    Circle circle = (v2, shape.Radius);
                    return RayCastCircle(input, circle);
                }

                // ray starts inside capsule . no hit
                return output;
            }

            // Perpendicular to capsule axis, pointing right
            Vec2 n = (a.Y, -a.X);

            var (u, rayLength) = d.GetLengthAndNormalize();

            // Intersect ray with infinite length capsule
            // v1 + radius * n + s1 * a = p1 + s2 * u
            // v1 - radius * n + s1 * a = p1 + s2 * u

            // s1 * a - s2 * u = b
            // b = q - radius * ap
            // or
            // b = q + radius * ap

            // Cramer's rule [a -u]
            float den = -a.X * u.Y + u.X * a.Y;
            if (-float.Epsilon < den && den < float.Epsilon)
            {
                // Ray is parallel to capsule and outside infinite length capsule
                return output;
            }

            Vec2 b1 = B2Math.MulSub(q, radius, n);
            Vec2 b2 = B2Math.MulAdd(q, radius, n);

            float invDen = 1.0f / den;

            // Cramer's rule [a b1]
            float s21 = (a.X * b1.Y - b1.X * a.Y) * invDen;

            // Cramer's rule [a b2]
            float s22 = (a.X * b2.Y - b2.X * a.Y) * invDen;

            float s2;
            Vec2 b;
            if (s21 < s22)
            {
                s2 = s21;
                b = b1;
            }
            else
            {
                s2 = s22;
                b = b2;
                n = B2Math.Neg(n);
            }

            if (s2 < 0.0f || input.MaxFraction * rayLength < s2)
            {
                return output;
            }

            // Cramer's rule [b -u]
            float s1 = (-b.X * u.Y + u.X * b.Y) * invDen;

            if (s1 < 0.0f)
            {
                // ray passes behind capsule segment
                Circle circle = (v1, shape.Radius);
                return RayCastCircle(input, circle);
            }
            else if (capsuleLength < s1)
            {
                // ray passes ahead of capsule segment
                Circle circle = (v2, shape.Radius);
                return RayCastCircle(input, circle);
            }
            else
            {
                // ray hits capsule side
                output.Fraction = s2 / rayLength;
                output.Point = B2Math.Add(B2Math.Lerp(v1, v2, s1 / capsuleLength), B2Math.MulSV(shape.Radius, n));
                output.Normal = n;
                output.Hit = true;
                return output;
            }
        }

        /// <summary>
        /// Ray vs line segment
        /// 对线段发出射线
        /// </summary>
        /// <param name="input"></param>
        /// <param name="shape"></param>
        /// <param name="oneSided"></param>
        /// <returns></returns>
        public static CastOutput RayCastSegment(RayCastInput input, Segment shape, bool oneSided)
        {
            if (oneSided)
            {
                // Skip left-side collision
                float offset = B2Math.Cross(B2Math.Sub(input.Origin, shape.Point1), B2Math.Sub(shape.Point2, shape.Point1));
                if (offset < 0.0f)
                {
                    return new();
                }
            }

            // Put the ray into the edge's frame of reference.
            Vec2 p1 = input.Origin;
            Vec2 d = input.Translation;

            Vec2 v1 = shape.Point1;
            Vec2 v2 = shape.Point2;
            Vec2 e = B2Math.Sub(v2, v1);

            CastOutput output = new();

            var (eUnit, length) = e.GetLengthAndNormalize();
            if (length == 0.0f)
            {
                return output;
            }

            // Normal points to the right, looking from v1 towards v2
            Vec2 normal = B2Math.RightPerp(eUnit);

            // Intersect ray with infinite segment using normal
            // Similar to intersecting a ray with an infinite plane
            // p = p1 + t * d
            // dot(normal, p - v1) = 0
            // dot(normal, p1 - v1) + t * dot(normal, d) = 0
            float numerator = B2Math.Dot(normal, B2Math.Sub(v1, p1));
            float denominator = B2Math.Dot(normal, d);

            if (denominator == 0.0f)
            {
                // parallel
                return output;
            }

            float t = numerator / denominator;
            if (t < 0.0f || input.MaxFraction < t)
            {
                // out of ray range
                return output;
            }

            // Intersection point on infinite segment
            Vec2 p = B2Math.MulAdd(p1, t, d);

            // Compute position of p along segment
            // p = v1 + s * e
            // s = dot(p - v1, e) / dot(e, e)

            float s = B2Math.Dot(B2Math.Sub(p, v1), eUnit);
            if (s < 0.0f || length < s)
            {
                // out of segment range
                return output;
            }

            if (numerator > 0.0f)
            {
                normal = B2Math.Neg(normal);
            }

            output.Fraction = t;
            output.Point = B2Math.MulAdd(p1, t, d);
            output.Normal = normal;
            output.Hit = true;

            return output;
        }

        /// <summary>
        /// 对多边形发出射线
        /// </summary>
        /// <param name="input"></param>
        /// <param name="shape"></param>
        /// <returns></returns>
        public static CastOutput RayCastPolygon(RayCastInput input, Polygon shape)
        {
            Debug.Assert(IsValidRay(input));

            if (shape.Radius == 0.0f)
            {
                // Put the ray into the polygon's frame of reference.
                Vec2 p1 = input.Origin;
                Vec2 d = input.Translation;

                float lower = 0.0f, upper = input.MaxFraction;

                int index = -1;

                CastOutput output = new();

                for (int i = 0; i < shape.Count; ++i)
                {
                    // p = p1 + a * d
                    // dot(normal, p - v) = 0
                    // dot(normal, p1 - v) + a * dot(normal, d) = 0
                    float numerator = B2Math.Dot(shape.Normals[i], B2Math.Sub(shape.Vertices[i], p1));
                    float denominator = B2Math.Dot(shape.Normals[i], d);

                    if (denominator == 0.0f)
                    {
                        if (numerator < 0.0f)
                        {
                            return output;
                        }
                    }
                    else
                    {
                        // Note: we want this predicate without division:
                        // lower < numerator / denominator, where denominator < 0
                        // Since denominator < 0, we have to flip the inequality:
                        // lower < numerator / denominator <==> denominator * lower > numerator.
                        if (denominator < 0.0f && numerator < lower * denominator)
                        {
                            // Increase lower.
                            // The segment enters this half-space.
                            lower = numerator / denominator;
                            index = i;
                        }
                        else if (denominator > 0.0f && numerator < upper * denominator)
                        {
                            // Decrease upper.
                            // The segment exits this half-space.
                            upper = numerator / denominator;
                        }
                    }

                    // The use of epsilon here causes the Debug.Assert on lower to trip
                    // in some cases. Apparently the use of epsilon was to make edge
                    // shapes work, but now those are handled separately.
                    // if (upper < lower - b2_epsilon)
                    if (upper < lower)
                    {
                        return output;
                    }
                }

                Debug.Assert(0.0f <= lower && lower <= input.MaxFraction);

                if (index >= 0)
                {
                    output.Fraction = lower;
                    output.Normal = shape.Normals[index];
                    output.Point = B2Math.MulAdd(p1, lower, d);
                    output.Hit = true;
                }

                return output;
            }

            // TODO_ERIN this is not working for ray vs box (zero radii)
            ShapeCastPairInput castInput;
            castInput.ProxyA = DistanceFunc.MakeProxy(shape.Vertices, shape.Count, shape.Radius);
            castInput.ProxyB = DistanceFunc.MakeProxy(input.Origin,  0.0f);
            castInput.TransformA = Transform.Identity;
            castInput.TransformB = Transform.Identity;
            castInput.TranslationB = input.Translation;
            castInput.MaxFraction = input.MaxFraction;
            return DistanceFunc.ShapeCast(castInput);
        }

        public static CastOutput ShapeCastCircle(ShapeCastInput input, Circle shape)
        {
            ShapeCastPairInput pairInput;
            pairInput.ProxyA = DistanceFunc.MakeProxy(shape.Center,  shape.Radius);
            pairInput.ProxyB = DistanceFunc.MakeProxy(input.Points, input.Count, input.Radius);
            pairInput.TransformA = Transform.Identity;
            pairInput.TransformB = Transform.Identity;
            pairInput.TranslationB = input.Translation;
            pairInput.MaxFraction = input.MaxFraction;

            CastOutput output = DistanceFunc.ShapeCast(pairInput);
            return output;
        }

        public static CastOutput ShapeCastCapsule(ShapeCastInput input, Capsule shape)
        {
            ShapeCastPairInput pairInput;
            pairInput.ProxyA = DistanceFunc.MakeProxy(shape.Points, 2, shape.Radius);
            pairInput.ProxyB = DistanceFunc.MakeProxy(input.Points, input.Count, input.Radius);
            pairInput.TransformA = Transform.Identity;
            pairInput.TransformB = Transform.Identity;
            pairInput.TranslationB = input.Translation;
            pairInput.MaxFraction = input.MaxFraction;

            CastOutput output = DistanceFunc.ShapeCast(pairInput);
            return output;
        }

        public static CastOutput ShapeCastSegment(ShapeCastInput input, Segment shape)
        {
            ShapeCastPairInput pairInput;
            pairInput.ProxyA = DistanceFunc.MakeProxy(shape.Points, 2, 0.0f);
            pairInput.ProxyB = DistanceFunc.MakeProxy(input.Points, input.Count, input.Radius);
            pairInput.TransformA = Transform.Identity;
            pairInput.TransformB = Transform.Identity;
            pairInput.TranslationB = input.Translation;
            pairInput.MaxFraction = input.MaxFraction;

            CastOutput output = DistanceFunc.ShapeCast(pairInput);
            return output;
        }

        public static CastOutput ShapeCastPolygon(ShapeCastInput input, Polygon shape)
        {
            ShapeCastPairInput pairInput;
            pairInput.ProxyA = DistanceFunc.MakeProxy(shape.Vertices, shape.Count, shape.Radius);
            pairInput.ProxyB = DistanceFunc.MakeProxy(input.Points, input.Count, input.Radius);
            pairInput.TransformA = Transform.Identity;
            pairInput.TransformB = Transform.Identity;
            pairInput.TranslationB = input.Translation;
            pairInput.MaxFraction = input.MaxFraction;

            CastOutput output = DistanceFunc.ShapeCast(pairInput);
            return output;
        }
    }
}