using System.Diagnostics;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Contacts
{
    public class ChainAndCircleContact : Contact
    {
        private ChainAndCircleContact(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB) : base(
            fixtureA,
            indexA,
            fixtureB,
            indexB)
        {
            Debug.Assert(FixtureA.GetShapeType() == ShapeType.Chain);
            Debug.Assert(FixtureB.GetShapeType() == ShapeType.Circle);
        }

        public static Contact Create(
            Fixture fixtureA,
            int     indexA,
            Fixture fixtureB,
            int     indexB)
        {
            return new ChainAndCircleContact(fixtureA, indexA, fixtureB, indexB);
        }

        public static void Destroy(Contact contact)
        {
            // Todo return to pool
        }

        /// <inheritdoc />
        internal override void Evaluate(ref Manifold manifold, in Transform xfA, Transform xfB)
        {
            var chain = (ChainShape) FixtureA.GetShape();

            chain.GetChildEdge(out var edge, IndexA);
            CollisionUtils.CollideEdgeAndCircle(
                ref manifold,
                edge,
                xfA,
                (CircleShape) FixtureB.GetShape(),
                xfB);
        }
    }
}