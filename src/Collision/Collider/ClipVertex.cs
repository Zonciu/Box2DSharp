using System.Numerics;

namespace Box2DSharp.Collision.Collider
{
    /// Used for computing contact manifolds.
    public struct ClipVertex
    {
        public Vector2 v;

        public ContactID id;
    };
}