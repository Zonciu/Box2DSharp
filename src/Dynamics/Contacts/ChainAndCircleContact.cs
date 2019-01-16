using System.Diagnostics;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Microsoft.Extensions.ObjectPool;

namespace Box2DSharp.Dynamics.Contacts
{
    public class ChainAndCircleContact : Contact
    {
        private static readonly ObjectPool<ChainAndCircleContact> _pool =
            new DefaultObjectPool<ChainAndCircleContact>(new ContactPoolPolicy<ChainAndCircleContact>());

        internal static Contact Create(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB)
        {
            Debug.Assert(fixtureA.ShapeType == ShapeType.Chain);
            Debug.Assert(fixtureB.ShapeType == ShapeType.Circle);
            var contact = _pool.Get();
            contact.Initialize(fixtureA, indexA, fixtureB, indexB);
            return contact;
        }

        public static void Destroy(Contact contact)
        {
            _pool.Return((ChainAndCircleContact) contact);
        }

        internal override void Evaluate(ref Manifold manifold, in Transform xfA, Transform xfB)
        {
            var chain = (ChainShape) FixtureA.Shape;

            chain.GetChildEdge(out var edge, ChildIndexA);
            CollisionUtils.CollideEdgeAndCircle(
                ref manifold,
                edge,
                xfA,
                (CircleShape) FixtureB.Shape,
                xfB);
        }
    }
}