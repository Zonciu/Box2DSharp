using System;

namespace Box2DSharp
{
    /// <summary>
    /// Axis-aligned bounding box (16 bytes)
    /// 轴对齐包围框
    /// </summary>
    public struct AABB
    {
        public Vec2 LowerBound;

        public Vec2 UpperBound;

        /// <summary>
        /// 
        /// </summary>
        /// <param name="lowerBound"></param>
        /// <param name="upperBound"></param>
        public AABB(Vec2 lowerBound, Vec2 upperBound)
        {
            LowerBound = lowerBound;
            UpperBound = upperBound;
        }

        public AABB(float lowerX, float lowerY, float upperX, float upperY)
        {
            LowerBound = new(lowerX, lowerY);
            UpperBound = new(upperX, upperY);
        }

        public override string ToString()
        {
            return $"({LowerBound},{UpperBound})";
        }

        public static implicit operator AABB((Vec2 lowerBound, Vec2 upperBound) tuple)
        {
            return new AABB
            {
                LowerBound = tuple.lowerBound,
                UpperBound = tuple.upperBound
            };
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public bool IsValid
        {
            get
            {
                var d = B2Math.Sub(UpperBound, LowerBound);
                var valid = d is { X: >= 0.0f, Y: >= 0.0f };
                valid = valid && B2Math.Vec2_IsValid(LowerBound) && B2Math.Vec2_IsValid(UpperBound);
                return valid;
            }
        }

        /// <summary>
        /// From Real-time Collision Detection, p179.
        /// 射线检测
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <returns></returns>
        public CastOutput RayCast(Vec2 p1, Vec2 p2)
        {
            // Radius not handled
            CastOutput output = new();

            var tmin = -float.MaxValue;
            var tmax = float.MaxValue;

            Vec2 p = p1;
            Vec2 d = B2Math.Sub(p2, p1);
            Vec2 absD = B2Math.Abs(d);

            Vec2 normal = Vec2.Zero;

            // x-coordinate
            if (absD.X < float.Epsilon)
            {
                // parallel
                if (p.X < LowerBound.X || UpperBound.X < p.X)
                {
                    return output;
                }
            }
            else
            {
                float inv_d = 1.0f / d.X;
                float t1 = (LowerBound.X - p.X) * inv_d;
                float t2 = (UpperBound.X - p.X) * inv_d;

                // Sign of the normal vector.
                float s = -1.0f;

                if (t1 > t2)
                {
                    (t1, t2) = (t2, t1);
                    s = 1.0f;
                }

                // Push the min up
                if (t1 > tmin)
                {
                    normal.Y = 0.0f;
                    normal.X = s;
                    tmin = t1;
                }

                // Pull the max down
                tmax = Math.Min(tmax, t2);

                if (tmin > tmax)
                {
                    return output;
                }
            }

            // y-coordinate
            if (absD.Y < float.Epsilon)
            {
                // parallel
                if (p.Y < LowerBound.Y || UpperBound.Y < p.Y)
                {
                    return output;
                }
            }
            else
            {
                float inv_d = 1.0f / d.Y;
                float t1 = (LowerBound.Y - p.Y) * inv_d;
                float t2 = (UpperBound.Y - p.Y) * inv_d;

                // Sign of the normal vector.
                float s = -1.0f;

                if (t1 > t2)
                {
                    (t1, t2) = (t2, t1);
                    s = 1.0f;
                }

                // Push the min up
                if (t1 > tmin)
                {
                    normal.X = 0.0f;
                    normal.Y = s;
                    tmin = t1;
                }

                // Pull the max down
                tmax = Math.Min(tmax, t2);

                if (tmin > tmax)
                {
                    return output;
                }
            }

            // Does the ray start inside the box?
            // Does the ray intersect beyond the max fraction?
            if (tmin is < 0.0f or > 1.0f)
            {
                return output;
            }

            // Intersection.
            output.Fraction = tmin;
            output.Normal = normal;
            output.Point = B2Math.Lerp(p1, p2, tmin);
            output.Hit = true;
            return output;
        }

        /// <summary>
        /// Get the perimeter length
        /// 获取周长
        /// </summary>
        /// <returns></returns>
        public float Perimeter => 2.0f * (UpperBound.X - LowerBound.X + UpperBound.Y - LowerBound.Y);

        /// <summary>
        /// Enlarge a to contain b
        /// 扩大本包围框到可以包含b
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns>true if the AABB grew</returns>
        public static bool EnlargeAABB(ref AABB a, ref AABB b)
        {
            var changed = false;
            if (b.LowerBound.X < a.LowerBound.X)
            {
                a.LowerBound.X = b.LowerBound.X;
                changed = true;
            }

            if (b.LowerBound.Y < a.LowerBound.Y)
            {
                a.LowerBound.Y = b.LowerBound.Y;
                changed = true;
            }

            if (a.UpperBound.X < b.UpperBound.X)
            {
                a.UpperBound.X = b.UpperBound.X;
                changed = true;
            }

            if (a.UpperBound.Y < b.UpperBound.Y)
            {
                a.UpperBound.Y = b.UpperBound.Y;
                changed = true;
            }

            return changed;
        }

        /// <summary>
        /// Do a and b overlap
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static bool Overlaps(in AABB a, in AABB b)
        {
            if (b.LowerBound.X - a.UpperBound.X > 0.0f || b.LowerBound.Y - a.UpperBound.Y > 0.0f || a.LowerBound.X - b.UpperBound.X > 0 || a.LowerBound.Y - b.UpperBound.Y > 0)
            {
                return false;
            }

            return true;
        }

        public static bool Contains(in AABB a, in AABB b)
        {
            return a.LowerBound.X <= b.LowerBound.X
                && a.LowerBound.Y <= b.LowerBound.Y
                && b.UpperBound.X <= a.UpperBound.X
                && b.UpperBound.Y <= a.UpperBound.Y;
        }
    }
}