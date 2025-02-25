using System;
using System.Diagnostics;

namespace Box2DSharp
{
    public static class HullFunc
    {
        // quickhull recursion
        public static Hull RecurseHull(Vec2 p1, Vec2 p2, Span<Vec2> ps, int count)
        {
            Hull hull = new();
            hull.Count = 0;

            if (count == 0)
            {
                return hull;
            }

            // create an edge vector pointing from p1 to p2
            Vec2 e = B2Math.Sub(p2, p1).Normalize;

            // discard points left of e and find point furthest to the right of e
            Span<Vec2> rightPoints = stackalloc Vec2[Core.MaxPolygonVertices];
            int rightCount = 0;

            int bestIndex = 0;
            float bestDistance = B2Math.Cross(B2Math.Sub(ps[bestIndex], p1), e);
            if (bestDistance > 0.0f)
            {
                rightPoints[rightCount++] = ps[bestIndex];
            }

            for (int i = 1; i < count; ++i)
            {
                float distance = B2Math.Cross(B2Math.Sub(ps[i], p1), e);
                if (distance > bestDistance)
                {
                    bestIndex = i;
                    bestDistance = distance;
                }

                if (distance > 0.0f)
                {
                    rightPoints[rightCount++] = ps[i];
                }
            }

            if (bestDistance < 2.0f * Core.LinearSlop)
            {
                return hull;
            }

            Vec2 bestPoint = ps[bestIndex];

            // compute hull to the right of p1-bestPoint
            Hull hull1 = RecurseHull(p1, bestPoint, rightPoints, rightCount);

            // compute hull to the right of bestPoint-p2
            Hull hull2 = RecurseHull(bestPoint, p2, rightPoints, rightCount);

            // stitch together hulls
            for (int i = 0; i < hull1.Count; ++i)
            {
                hull.Points[hull.Count++] = hull1.Points[i];
            }

            hull.Points[hull.Count++] = bestPoint;

            for (int i = 0; i < hull2.Count; ++i)
            {
                hull.Points[hull.Count++] = hull2.Points[i];
            }

            Debug.Assert(hull.Count < Core.MaxPolygonVertices);

            return hull;
        }

        // quickhull algorithm
        // - merges vertices based on Core.b2_linearSlop
        // - removes collinear points using Core.b2_linearSlop
        // - returns an empty hull if it fails
        public static Hull ComputeHull(Span<Vec2> points, int count)
        {
            Hull hull = new();
            hull.Count = 0;

            if (count < 3 || count > Core.MaxPolygonVertices)
            {
                // check your data
                return hull;
            }

            count = Math.Min(count, Core.MaxPolygonVertices);

            AABB aabb = new(new(float.MaxValue, float.MaxValue), new(-float.MaxValue, -float.MaxValue));

            // Perform aggressive point welding. First point always remains.
            // Also compute the bounding box for later.
            Span<Vec2> ps = stackalloc Vec2[Core.MaxPolygonVertices];
            int n = 0;
            float linearSlop = Core.LinearSlop;
            float tolSqr = 16.0f * linearSlop * linearSlop;
            for (int i = 0; i < count; ++i)
            {
                aabb.LowerBound = B2Math.Min(aabb.LowerBound, points[i]);
                aabb.UpperBound = B2Math.Max(aabb.UpperBound, points[i]);

                Vec2 vi = points[i];

                bool unique = true;
                for (int j = 0; j < i; ++j)
                {
                    Vec2 vj = points[j];

                    float distSqr = B2Math.DistanceSquared(vi, vj);
                    if (distSqr < tolSqr)
                    {
                        unique = false;
                        break;
                    }
                }

                if (unique)
                {
                    ps[n++] = vi;
                }
            }

            if (n < 3)
            {
                // all points very close together, check your data and check your scale
                return hull;
            }

            // Find an extreme point as the first point on the hull
            Vec2 c = B2Math.AABB_Center(aabb);
            int f1 = 0;
            float dsq1 = B2Math.DistanceSquared(c, ps[f1]);
            for (int i = 1; i < n; ++i)
            {
                float dsq = B2Math.DistanceSquared(c, ps[i]);
                if (dsq > dsq1)
                {
                    f1 = i;
                    dsq1 = dsq;
                }
            }

            // remove p1 from working set
            Vec2 p1 = ps[f1];
            ps[f1] = ps[n - 1];
            n = n - 1;

            int f2 = 0;
            float dsq2 = B2Math.DistanceSquared(p1, ps[f2]);
            for (int i = 1; i < n; ++i)
            {
                float dsq = B2Math.DistanceSquared(p1, ps[i]);
                if (dsq > dsq2)
                {
                    f2 = i;
                    dsq2 = dsq;
                }
            }

            // remove p2 from working set
            Vec2 p2 = ps[f2];
            ps[f2] = ps[n - 1];
            n = n - 1;

            // split the points into points that are left and right of the line p1-p2.
            Span<Vec2> rightPoints = stackalloc Vec2[Core.MaxPolygonVertices - 2];
            int rightCount = 0;

            Span<Vec2> leftPoints = stackalloc Vec2[Core.MaxPolygonVertices - 2];
            int leftCount = 0;

            Vec2 e = B2Math.Sub(p2, p1).Normalize;

            for (int i = 0; i < n; ++i)
            {
                float d = B2Math.Cross(B2Math.Sub(ps[i], p1), e);

                // slop used here to skip points that are very close to the line p1-p2
                if (d >= 2.0f * linearSlop)
                {
                    rightPoints[rightCount++] = ps[i];
                }
                else if (d <= -2.0f * linearSlop)
                {
                    leftPoints[leftCount++] = ps[i];
                }
            }

            // compute hulls on right and left
            Hull hull1 = RecurseHull(p1, p2, rightPoints, rightCount);
            Hull hull2 = RecurseHull(p2, p1, leftPoints, leftCount);

            if (hull1.Count == 0 && hull2.Count == 0)
            {
                // all points collinear
                return hull;
            }

            // stitch hulls together, preserving CCW winding order
            hull.Points[hull.Count++] = p1;

            for (int i = 0; i < hull1.Count; ++i)
            {
                hull.Points[hull.Count++] = hull1.Points[i];
            }

            hull.Points[hull.Count++] = p2;

            for (int i = 0; i < hull2.Count; ++i)
            {
                hull.Points[hull.Count++] = hull2.Points[i];
            }

            Debug.Assert(hull.Count <= Core.MaxPolygonVertices);

            // merge collinear
            bool searching = true;
            while (searching && hull.Count > 2)
            {
                searching = false;

                for (int i = 0; i < hull.Count; ++i)
                {
                    int i1 = i;
                    int i2 = (i + 1) % hull.Count;
                    int i3 = (i + 2) % hull.Count;

                    Vec2 s1 = hull.Points[i1];
                    Vec2 s2 = hull.Points[i2];
                    Vec2 s3 = hull.Points[i3];

                    // unit edge vector for s1-s3
                    Vec2 r = B2Math.Sub(s3, s1).Normalize;

                    float distance = B2Math.Cross(B2Math.Sub(s2, s1), r);
                    if (distance <= 2.0f * linearSlop)
                    {
                        // remove midpoint from hull
                        for (int j = i2; j < hull.Count - 1; ++j)
                        {
                            hull.Points[j] = hull.Points[j + 1];
                        }

                        hull.Count -= 1;

                        // continue searching for collinear points
                        searching = true;

                        break;
                    }
                }
            }

            if (hull.Count < 3)
            {
                // all points collinear, shouldn't be reached since this was validated above
                hull.Count = 0;
            }

            return hull;
        }

        public static bool ValidateHull(Hull hull)
        {
            if (hull.Count < 3 || Core.MaxPolygonVertices < hull.Count)
            {
                return false;
            }

            // test that every point is behind every edge
            for (int i = 0; i < hull.Count; ++i)
            {
                // create an edge vector
                int i1 = i;
                int i2 = i < hull.Count - 1 ? i1 + 1 : 0;
                Vec2 p = hull.Points[i1];
                Vec2 e = B2Math.Sub(hull.Points[i2], p).Normalize;

                for (int j = 0; j < hull.Count; ++j)
                {
                    // skip points that subtend the current edge
                    if (j == i1 || j == i2)
                    {
                        continue;
                    }

                    float distance = B2Math.Cross(B2Math.Sub(hull.Points[j], p), e);
                    if (distance >= 0.0f)
                    {
                        return false;
                    }
                }
            }

            // test for collinear points
            float linearSlop = Core.LinearSlop;
            for (int i = 0; i < hull.Count; ++i)
            {
                int i1 = i;
                int i2 = (i + 1) % hull.Count;
                int i3 = (i + 2) % hull.Count;

                Vec2 p1 = hull.Points[i1];
                Vec2 p2 = hull.Points[i2];
                Vec2 p3 = hull.Points[i3];

                Vec2 e = B2Math.Sub(p3, p1).Normalize;

                float distance = B2Math.Cross(B2Math.Sub(p2, p1), e);
                if (distance <= linearSlop)
                {
                    // p1-p2-p3 are collinear
                    return false;
                }
            }

            return true;
        }
    }
}