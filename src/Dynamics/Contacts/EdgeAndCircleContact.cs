using System.Diagnostics;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Contacts
{
    /// <summary>
    ///     边缘与圆接触
    /// </summary>
    public class EdgeAndCircleContact : Contact
    {
        private EdgeAndCircleContact(Fixture fixtureA, Fixture fixtureB) : base(fixtureA, 0, fixtureB, 0)
        {
            Debug.Assert(FixtureA.GetShapeType() == ShapeType.Edge);
            Debug.Assert(FixtureB.GetShapeType() == ShapeType.Circle);
        }

        public static Contact Create(
            Fixture fixtureA,
            int       indexA,
            Fixture fixtureB,
            int       indexB)
        {
            return new CircleContact(fixtureA, fixtureB);
        }

        internal static void Destroy(Contact contact)
        { }

        /// <inheritdoc />
        internal override void Evaluate(ref Manifold manifold, in Transform xfA, Transform xfB)
        {
            CollisionUtils.CollideEdgeAndCircle(
                ref manifold,
                (EdgeShape) FixtureA.GetShape(),
                xfA,
                (CircleShape) FixtureB.GetShape(),
                xfB);
        }
    }
}