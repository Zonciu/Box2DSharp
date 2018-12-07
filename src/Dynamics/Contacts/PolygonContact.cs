using System.Diagnostics;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Microsoft.Extensions.ObjectPool;

namespace Box2DSharp.Dynamics.Contacts
{
    public class PolygonContact : Contact
    {
        private static readonly ObjectPool<PolygonContact> _pool =
            new DefaultObjectPool<PolygonContact>(new PoolPolicy());

        private class PoolPolicy : IPooledObjectPolicy<PolygonContact>
        {
            public PolygonContact Create()
            {
                return new PolygonContact();
            }

            public bool Return(PolygonContact obj)
            {
                obj.Reset();
                return true;
            }
        }

        internal static Contact Create(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB)
        {
            Debug.Assert(fixtureA.GetShapeType() == ShapeType.Polygon);
            Debug.Assert(fixtureB.GetShapeType() == ShapeType.Polygon);
            var contact = _pool.Get();
            contact.Initialize(fixtureA, 0, fixtureB, 0);
            return contact;
        }

        public static void Destroy(Contact contact)
        {
            _pool.Return((PolygonContact) contact);
        }

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