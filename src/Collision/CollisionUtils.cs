using System.Numerics;
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
            var distance0 = MathUtils.Dot(normal, vIn[0].Vector) - offset;
            var distance1 = MathUtils.Dot(normal, vIn[1].Vector) - offset;

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
                vOut[numOut].Vector = vIn[0].Vector + interp * (vIn[1].Vector - vIn[0].Vector);

                // VertexA is hitting edgeB.
                vOut[numOut].Id.ContactFeature.IndexA = (byte) vertexIndexA;
                vOut[numOut].Id.ContactFeature.IndexB = vIn[0].Id.ContactFeature.IndexB;
                vOut[numOut].Id.ContactFeature.TypeA  = (byte) ContactFeature.FeatureType.Vertex;
                vOut[numOut].Id.ContactFeature.TypeB  = (byte) ContactFeature.FeatureType.Face;
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
            input.ProxyA.Set(shapeA, indexA);
            input.ProxyB.Set(shapeB, indexB);
            input.TransformA = xfA;
            input.TransformB = xfB;
            input.UseRadii   = true;

            var cache = new SimplexCache();

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