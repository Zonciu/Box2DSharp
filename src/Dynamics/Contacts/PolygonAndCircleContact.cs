using System.Diagnostics;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Microsoft.Extensions.ObjectPool;

namespace Box2DSharp.Dynamics.Contacts
{
    /// <summary>
    ///     多边形与圆接触
    /// </summary>
    public class PolygonAndCircleContact : Contact
    {
        private static readonly ObjectPool<PolygonAndCircleContact> _pool =
            new DefaultObjectPool<PolygonAndCircleContact>(new ContactPoolPolicy<PolygonAndCircleContact>());

        internal static Contact Create(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB)
        {
            Debug.Assert(fixtureA.ShapeType == ShapeType.Polygon);
            Debug.Assert(fixtureB.ShapeType == ShapeType.Circle);
            var contact = _pool.Get();
            contact.Initialize(fixtureA, 0, fixtureB, 0);
            return contact;
        }

        public static void Destroy(Contact contact)
        {
            _pool.Return((PolygonAndCircleContact) contact);
        }

        /// <inheritdoc />
        internal override void Evaluate(ref Manifold manifold, in Transform xfA, Transform xfB)
        {
            CollisionUtils.CollidePolygonAndCircle(
                ref manifold,
                (PolygonShape) FixtureA.Shape,
                xfA,
                (CircleShape) FixtureB.Shape,
                xfB);
        }

        private class PoolPolicy : IPooledObjectPolicy<PolygonAndCircleContact>
        {
            public PolygonAndCircleContact Create()
            {
                return new PolygonAndCircleContact();
            }

            public bool Return(PolygonAndCircleContact obj)
            {
                obj.Reset();
                return true;
            }
        }
    }
}