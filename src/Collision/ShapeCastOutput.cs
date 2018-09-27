using System.Numerics;

namespace Box2DSharp.Collision
{
    /// Output results for b2ShapeCast
    public struct ShapeCastOutput
    {
        public Vector2 point;

        public Vector2 normal;

        public float lambda;

        public int iterations;
    };
}