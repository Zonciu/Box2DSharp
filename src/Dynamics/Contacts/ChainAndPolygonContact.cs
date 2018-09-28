using System.Diagnostics;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Contacts
{
    public class ChainAndPolygonContact : Contact
    {
        internal static Contact Create(
            Fixture fixtureA,
            int       indexA,
            Fixture fixtureB,
            int       indexB)
        {
            return new ChainAndPolygonContact(fixtureA, indexA, fixtureB, indexB);
        }

        internal static void Destroy(Contact contact)
        { }

        private ChainAndPolygonContact(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB) : base(
            fixtureA,
            indexA,
            fixtureB,
            indexB)
        {
            Debug.Assert(FixtureA.GetShapeType() == ShapeType.Chain);
            Debug.Assert(FixtureB.GetShapeType() == ShapeType.Polygon);
        }

        /// <inheritdoc />
        internal override void Evaluate(ref Manifold manifold, in Transform xfA, Transform xfB)
        {
            var chain = (ChainShape) FixtureA.GetShape();

            chain.GetChildEdge(out var edge, IndexA);
            CollisionUtils.CollideEdgeAndPolygon(ref manifold, edge, xfA, (PolygonShape) FixtureB.GetShape(), xfB);
        }
    }
}