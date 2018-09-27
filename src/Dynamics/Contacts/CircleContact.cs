using System.Diagnostics;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Contacts
{
    internal class CircleContact : Contact
    {
        internal static Contact Create(
            Fixture fixtureA,
            int     indexA,
            Fixture fixtureB,
            int     indexB)
        {
            return new CircleContact(fixtureA, fixtureB);
        }

        internal static void Destroy(Contact contact)
        { }

        internal CircleContact(Fixture fixtureA, Fixture fixtureB) : base(fixtureA, 0, fixtureB, 0)
        {
            Debug.Assert(FixtureA.GetShapeType() == ShapeType.Circle);
            Debug.Assert(FixtureB.GetShapeType() == ShapeType.Circle);
        }

        /// <inheritdoc />
        internal override void Evaluate(ref Manifold manifold, in Transform xfA, Transform xfB)
        {
            Collision.CollisionUtils.CollideCircles(
                ref manifold,
                (CircleShape) FixtureA.GetShape(),
                xfA,
                (CircleShape) FixtureB.GetShape(),
                xfB);
        }
    }
}