using System.Numerics;

namespace Box2DSharp.Collision
{
    public struct SimplexVertex
    {
        public Vector2 wA; // support point in proxyA

        public Vector2 wB; // support point in proxyB

        public Vector2 w; // wB - wA

        public float a; // barycentric coordinate for closest point

        public int indexA; // wA index

        public int indexB; // wB index
    };
}