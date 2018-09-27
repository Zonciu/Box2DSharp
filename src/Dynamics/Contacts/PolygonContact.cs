using System.Diagnostics;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Contacts
{
    public class PolygonContact : Contact
    {
        private PolygonContact(Fixture fixtureA, Fixture fixtureB) : base(fixtureA, 0, fixtureB, 0)
        {
            Debug.Assert(FixtureA.GetShapeType() == ShapeType.Polygon);
            Debug.Assert(FixtureB.GetShapeType() == ShapeType.Polygon);
        }

        internal static Contact Create(
            Fixture fixtureA,
            int       indexA,
            Fixture fixtureB,
            int       indexB)
        {
            return new PolygonContact(fixtureA, fixtureB);
        }

        internal static void Destroy(Contact contact)
        { }

        /// <inheritdoc />
        internal override void Evaluate(ref Manifold manifold, in Transform xfA, Transform xfB)
        {
            CollisionUtils.CollidePolygons(
                ref manifold,
                (PolygonShape) FixtureA.GetShape(),
                xfA,
                (PolygonShape) FixtureB.GetShape(),
                xfB);
        }
    }
}