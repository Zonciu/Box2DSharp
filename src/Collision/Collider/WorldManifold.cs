using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Collision.Collider
{
    /// This is used to compute the current state of a contact manifold.
    public class WorldManifold
    {
        public static WorldManifold Create()
        {
            return new WorldManifold()
            {
                points      = new Vector2[Settings.MaxManifoldPoints],
                separations = new float[Settings.MaxManifoldPoints]
            };
        }

        /// Evaluate the manifold with supplied transforms. This assumes
        /// modest motion from the original state. This does not change the
        /// point count, impulses, etc. The radii must come from the shapes
        /// that generated the manifold.
        public void Initialize(
            in Manifold  manifold,
            in Transform xfA,
            float        radiusA,
            in Transform xfB,
            float        radiusB)
        {
            if (manifold.PointCount == 0)
            {
                return;
            }

            switch (manifold.Type)
            {
            case ManifoldType.Circles:
            {
                normal.Set(1.0f, 0.0f);
                var pointA = MathUtils.Mul(xfA, manifold.LocalPoint);
                var pointB = MathUtils.Mul(xfB, manifold.Points[0].localPoint);
                if (MathUtils.DistanceSquared(pointA, pointB) > Settings.Epsilon * Settings.Epsilon)
                {
                    normal = pointB - pointA;
                    normal.Normalize();
                }

                var cA = pointA + radiusA * normal;
                var cB = pointB - radiusB * normal;
                points[0]      = 0.5f * (cA + cB);
                separations[0] = MathUtils.Dot(cB - cA, normal);
            }
                break;

            case ManifoldType.FaceA:
            {
                normal = MathUtils.Mul(xfA.Rotation, manifold.LocalNormal);
                var planePoint = MathUtils.Mul(xfA, manifold.LocalPoint);

                for (var i = 0; i < manifold.PointCount; ++i)
                {
                    var clipPoint = MathUtils.Mul(xfB, manifold.Points[i].localPoint);
                    var cA        = clipPoint + (radiusA - MathUtils.Dot(clipPoint - planePoint, normal)) * normal;
                    var cB        = clipPoint - radiusB * normal;
                    points[i]      = 0.5f * (cA + cB);
                    separations[i] = MathUtils.Dot(cB - cA, normal);
                }
            }
                break;

            case ManifoldType.FaceB:
            {
                normal = MathUtils.Mul(xfB.Rotation, manifold.LocalNormal);
                var planePoint = MathUtils.Mul(xfB, manifold.LocalPoint);

                for (var i = 0; i < manifold.PointCount; ++i)
                {
                    var clipPoint = MathUtils.Mul(xfA, manifold.Points[i].localPoint);
                    var cB        = clipPoint + (radiusB - MathUtils.Dot(clipPoint - planePoint, normal)) * normal;
                    var cA        = clipPoint - radiusA * normal;
                    points[i]      = 0.5f * (cA + cB);
                    separations[i] = MathUtils.Dot(cA - cB, normal);
                }

                // Ensure normal points from A to B.
                normal = -normal;
            }
                break;
            }
        }

        /// world vector pointing from A to B
        public Vector2 normal;

        /// world contact point (point of intersection)
        public Vector2[] points;

        /// a negative value indicates overlap, in meters
        public float[] separations;
    };
}