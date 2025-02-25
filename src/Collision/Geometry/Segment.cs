using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DSharp
{
    /// <summary>
    /// A line segment with two-sided collision.(16 bytes)
    /// </summary>
    public struct Segment
    {
        /// <summary>
        /// The first point
        /// </summary>
        public Vec2 Point1;

        /// <summary>
        /// The second point
        /// </summary>
        public Vec2 Point2;

        public Span<Vec2> Points
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => MemoryMarshal.CreateSpan(ref Point1, 2);
        }

        public Segment(Vec2 point1, Vec2 point2)
        {
            Point1 = point1;
            Point2 = point2;
        }

        public static implicit operator Segment((Vec2 point1, Vec2 point2) tuple)
        {
            return new Segment(tuple.point1, tuple.point2);
        }
    }
}