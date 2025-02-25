using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DSharp
{
    /// <summary>
    /// A solid capsule can be viewed as two semicircles connected by a rectangle.(20 bytes)
    /// </summary>
    public struct Capsule
    {
        /// <summary>
        /// Local center of the first semicircle
        /// </summary>
        public Vec2 Center1;

        /// <summary>
        /// Local center of the second semicircle
        /// </summary>
        public Vec2 Center2;

        /// <summary>
        /// The radius of the semicircles
        /// </summary>
        public float Radius;

        public Span<Vec2> Points
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => MemoryMarshal.CreateSpan(ref Center1, 2);
        }

        public Capsule(Vec2 center1, Vec2 center2, float radius)
        {
            Center1 = center1;
            Center2 = center2;
            Radius = radius;
        }

        public static implicit operator Capsule((Vec2 center1, Vec2 center2, float radius) tuple)
        {
            return new Capsule(tuple.center1, tuple.center2, tuple.radius);
        }
    }
}