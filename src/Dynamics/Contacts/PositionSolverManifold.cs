using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Contacts
{
    public class PositionSolverManifold
    {
        public void Initialize(in ContactPositionConstraint pc, in Transform xfA, in Transform xfB, int index)
        {
            Debug.Assert(pc.pointCount > 0);

            switch (pc.type)
            {
            case ManifoldType.Circles:
            {
                Vector2 pointA = MathUtils.Mul(xfA, pc.localPoint);
                Vector2 pointB = MathUtils.Mul(xfB, pc.localPoints[0]);
                normal = pointB - pointA;
                normal.Normalize();
                point      = 0.5f * (pointA + pointB);
                separation = MathUtils.Dot(pointB - pointA, normal) - pc.radiusA - pc.radiusB;
            }
                break;

            case ManifoldType.FaceA:
            {
                normal = MathUtils.Mul(xfA.Rotation, pc.localNormal);
                Vector2 planePoint = MathUtils.Mul(xfA, pc.localPoint);

                Vector2 clipPoint = MathUtils.Mul(xfB, pc.localPoints[index]);
                separation = MathUtils.Dot(clipPoint - planePoint, normal) - pc.radiusA - pc.radiusB;
                point      = clipPoint;
            }
                break;

            case ManifoldType.FaceB:
            {
                normal = MathUtils.Mul(xfB.Rotation, pc.localNormal);
                Vector2 planePoint = MathUtils.Mul(xfB, pc.localPoint);

                Vector2 clipPoint = MathUtils.Mul(xfA, pc.localPoints[index]);
                separation = MathUtils.Dot(clipPoint - planePoint, normal) - pc.radiusA - pc.radiusB;
                point      = clipPoint;

                // Ensure normal points from A to B
                normal = -normal;
            }
                break;
            }
        }

        public Vector2 normal;

        public Vector2 point;

        public float separation;
    };
}