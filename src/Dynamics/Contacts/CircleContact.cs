using System.Diagnostics;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Microsoft.Extensions.ObjectPool;

namespace Box2DSharp.Dynamics.Contacts
{
    internal class CircleContact : Contact
    {
        private static readonly ObjectPool<CircleContact> _pool =
            new DefaultObjectPool<CircleContact>(new ContactPoolPolicy<CircleContact>());

        internal static Contact Create(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB)
        {
            Debug.Assert(fixtureA.ShapeType == ShapeType.Circle);
            Debug.Assert(fixtureB.ShapeType == ShapeType.Circle);
            var contact = _pool.Get();
            contact.Initialize(fixtureA, 0, fixtureB, 0);
            return contact;
        }

        public static void Destroy(Contact contact)
        {
            _pool.Return((CircleContact) contact);
        }

        internal override void Evaluate(ref Manifold manifold, in Transform xfA, Transform xfB)
        {
            CollisionUtils.CollideCircles(
                ref manifold,
                (CircleShape) FixtureA.Shape,
                xfA,
                (CircleShape) FixtureB.Shape,
                xfB);
        }
    }
}