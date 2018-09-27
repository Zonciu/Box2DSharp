using System.Numerics;
using System.Runtime.InteropServices;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;

namespace Box2DSharp.Collision
{
    public static partial class CollisionUtils
    {
        /// Clipping for contact manifolds.
        public static int ClipSegmentToLine(
            ref ClipVertex[] vOut,
            in  ClipVertex[] vIn,
            in  Vector2      normal,
            float            offset,
            int              vertexIndexA)
        {
            // Start with no output points
            var numOut = 0;

            // Calculate the distance of end points to the line
            var distance0 = MathUtils.Dot(normal, vIn[0].v) - offset;
            var distance1 = MathUtils.Dot(normal, vIn[1].v) - offset;

            // If the points are behind the plane
            if (distance0 <= 0.0f)
            {
                vOut[numOut++] = vIn[0];
            }

            if (distance1 <= 0.0f)
            {
                vOut[numOut++] = vIn[1];
            }

            // If the points are on different sides of the plane
            if (distance0 * distance1 < 0.0f)
            {
                // Find intersection point of edge and plane
                var interp = distance0 / (distance0 - distance1);
                vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);

                // VertexA is hitting edgeB.
                vOut[numOut].id.cf.IndexA = (byte) vertexIndexA;
                vOut[numOut].id.cf.IndexB = vIn[0].id.cf.IndexB;
                vOut[numOut].id.cf.TypeA  = (byte) ContactFeature.FeatureType.Vertex;
                vOut[numOut].id.cf.TypeB  = (byte) ContactFeature.FeatureType.Face;
                ++numOut;
            }

            return numOut;
        }

        /// Determine if two generic shapes overlap.
        public static bool TestOverlap(
            Shape        shapeA,
            int          indexA,
            Shape        shapeB,
            int          indexB,
            in Transform xfA,
            in Transform xfB)
        {
            var input = new DistanceInput();
            input.proxyA.Set(shapeA, indexA);
            input.proxyB.Set(shapeB, indexB);
            input.transformA = xfA;
            input.transformB = xfB;
            input.useRadii   = true;

            var cache = SimplexCache.Create();

            DistanceAlgorithm.Distance(out var output, ref cache, input);

            return output.Distance < 10.0f * Settings.Epsilon;
        }

        public static bool TestOverlap(in AABB a, AABB b)
        {
            var d1 = b.LowerBound - a.UpperBound;
            var d2 = a.LowerBound - b.UpperBound;

            if (d1.X > 0.0f || d1.Y > 0.0f)
            {
                return false;
            }

            if (d2.X > 0.0f || d2.Y > 0.0f)
            {
                return false;
            }

            return true;
        }
    }
}