using System.Diagnostics;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Contacts
{
    /// <summary>
    ///     边缘与多边形接触
    /// </summary>
    public class EdgeAndPolygonContact : Contact
    {
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
    }

    internal class EdgeAndPolygonContactFactory : IContactFactory
    {
        private readonly ObjectPool<EdgeAndPolygonContact> _pool =
            new ObjectPool<EdgeAndPolygonContact>(new ContactPoolPolicy<EdgeAndPolygonContact>());

        public Contact Create(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB)
        {
            Debug.Assert(fixtureA.ShapeType == ShapeType.Edge);
            Debug.Assert(fixtureB.ShapeType == ShapeType.Polygon);
            var contact = _pool.Get();
            contact.Initialize(fixtureA, 0, fixtureB, 0);
            return contact;
        }

        public void Destroy(Contact contact)
        {
            _pool.Return((EdgeAndPolygonContact) contact);
        }
    }
}