using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Collision.Collider
{
    /// This is used to compute the current state of a contact manifold.
    public struct WorldManifold
    {
        /// Evaluate the manifold with supplied transforms. This assumes
        /// modest motion from the original state. This does not change the
        /// point count, impulses, etc. The radii must come from the shapes
        /// that generated the manifold.
        public void Initialize(
            in Manifold manifold,
            in Transform xfA,
            float radiusA,
            in Transform xfB,
            float radiusB)
        {
            if (manifold.PointCount == 0)
            {
                return;
            }

            Points.Values = new Vector2[2];
            Separations.Values = new float[2];
            switch (manifold.Type)
            {
            case ManifoldType.Circles:
            {
                Normal.Set(1.0f, 0.0f);
                var pointA = MathUtils.Mul(xfA, manifold.LocalPoint);
                var pointB = MathUtils.Mul(xfB, manifold.Points.Values[0].LocalPoint);
                if (MathUtils.DistanceSquared(pointA, pointB) > Settings.Epsilon * Settings.Epsilon)
                {
                    Normal = pointB - pointA;
                    Normal = Vector2.Normalize(Normal);
                }

                var cA = pointA + radiusA * Normal;
                var cB = pointB - radiusB * Normal;
                Points.Values[0] = 0.5f * (cA + cB);
                Separations.Values[0] = MathUtils.Dot(cB - cA, Normal);
            }
                break;

            case ManifoldType.FaceA:
            {
                Normal = MathUtils.Mul(xfA.Rotation, manifold.LocalNormal);
                var planePoint = MathUtils.Mul(xfA, manifold.LocalPoint);

                for (var i = 0; i < manifold.PointCount; ++i)
                {
                    var clipPoint = MathUtils.Mul(xfB, manifold.Points.Values[i].LocalPoint);
                    var cA = clipPoint + (radiusA - MathUtils.Dot(clipPoint - planePoint, Normal)) * Normal;
                    var cB = clipPoint - radiusB * Normal;
                    Points.Values[i] = 0.5f * (cA + cB);
                    Separations.Values[i] = MathUtils.Dot(cB - cA, Normal);
                }
            }
                break;

            case ManifoldType.FaceB:
            {
                Normal = MathUtils.Mul(xfB.Rotation, manifold.LocalNormal);
                var planePoint = MathUtils.Mul(xfB, manifold.LocalPoint);

                for (var i = 0; i < manifold.PointCount; ++i)
                {
                    var clipPoint = MathUtils.Mul(xfA, manifold.Points.Values[i].LocalPoint);
                    var cB = clipPoint + (radiusB - MathUtils.Dot(clipPoint - planePoint, Normal)) * Normal;
                    var cA = clipPoint - radiusA * Normal;
                    Points.Values[i] = 0.5f * (cA + cB);
                    Separations.Values[i] = MathUtils.Dot(cA - cB, Normal);
                }

                // Ensure normal points from A to B.
                Normal = -Normal;
            }
                break;
            }
        }

        /// world vector pointing from A to B
        public Vector2 Normal;

        /// <summary>
        /// world contact point (point of intersection), size Settings.MaxManifoldPoints
        /// </summary>
        public FixedArray2<Vector2> Points;

        /// <summary>
        /// a negative value indicates overlap, in meters, size Settings.MaxManifoldPoints
        /// </summary>
        public FixedArray2<float> Separations;
    }
}