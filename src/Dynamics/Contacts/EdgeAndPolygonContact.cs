using System.Diagnostics;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Microsoft.Extensions.ObjectPool;

namespace Box2DSharp.Dynamics.Contacts
{
    /// <summary>
    ///     边缘与多边形接触
    /// </summary>
    public class EdgeAndPolygonContact : Contact
    {
        private static readonly ObjectPool<EdgeAndPolygonContact> _pool =
            new DefaultObjectPool<EdgeAndPolygonContact>(new ContactPoolPolicy<EdgeAndPolygonContact>());

        internal static Contact Create(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB)
        {
            Debug.Assert(fixtureA.ShapeType == ShapeType.Edge);
            Debug.Assert(fixtureB.ShapeType == ShapeType.Polygon);
            var contact = _pool.Get();
            contact.Initialize(fixtureA, 0, fixtureB, 0);
            return contact;
        }

        public static void Destroy(Contact contact)
        {
            _pool.Return((EdgeAndPolygonContact) contact);
        }

        /// <inheritdoc />
        internal override void Evaluate(ref Manifold manifold, in Transform xfA, Transform xfB)
        {
            CollisionUtils.CollideEdgeAndPolygon(
                ref manifold,
                (EdgeShape) FixtureA.Shape,
                xfA,
                (PolygonShape) FixtureB.Shape,
                xfB);
        }

        private class PoolPolicy : IPooledObjectPolicy<EdgeAndPolygonContact>
        {
            public EdgeAndPolygonContact Create()
            {
                return new EdgeAndPolygonContact();
            }

            public bool Return(EdgeAndPolygonContact obj)
            {
                obj.Reset();
                return true;
            }
        }
    }
}