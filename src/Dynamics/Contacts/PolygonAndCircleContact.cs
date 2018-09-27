using System.Diagnostics;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Contacts
{
    /// <summary>
    ///     多边形与圆接触
    /// </summary>
    public class PolygonAndCircleContact : Contact
    {
        private PolygonAndCircleContact(Fixture fixtureA, Fixture fixtureB) : base(fixtureA, 0, fixtureB, 0)
        {
            Debug.Assert(FixtureA.GetShapeType() == ShapeType.Polygon);
            Debug.Assert(FixtureB.GetShapeType() == ShapeType.Circle);
        }

        internal static Contact Create(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB)
        {
            return new PolygonAndCircleContact(fixtureA, fixtureB);
        }

        internal static void Destroy(Contact contact)
        { }

        /// <inheritdoc />
        internal override void Evaluate(ref Manifold manifold, in Transform xfA, Transform xfB)
        {
            CollisionUtils.CollidePolygonAndCircle(
                ref manifold,
                (PolygonShape) FixtureA.GetShape(),
                xfA,
                (CircleShape) FixtureB.GetShape(),
                xfB);
        }
    }
}