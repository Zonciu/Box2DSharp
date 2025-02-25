using System;

namespace Box2DSharp
{
    /// <summary>
    /// Shape id references a shape instance. This should be treated as an opaque handle.
    /// </summary>
    public struct ShapeId : IEquatable<ShapeId>
    {
        public int Index1;

        public ushort World0;

        public ushort Revision;

        public static ShapeId NullId = new();

        public bool IsNull => Index1 == 0;

        public bool IsNotNull => Index1 != 0;

        public static implicit operator ShapeId((int index1, ushort world0, ushort revision) tuple)
        {
            return new ShapeId(tuple.index1, tuple.world0, tuple.revision);
        }

        public ShapeId(int index1, ushort world0, ushort revision)
        {
            Index1 = index1;
            World0 = world0;
            Revision = revision;
        }

        public bool Equals(ShapeId other)
        {
            return Index1 == other.Index1
                && World0 == other.World0
                && Revision == other.Revision;
        }

        public override bool Equals(object? obj)
        {
            return obj is ShapeId other && Equals(other);
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(Index1, World0, Revision);
        }

        public static bool operator ==(ShapeId left, ShapeId right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(ShapeId left, ShapeId right)
        {
            return !(left == right);
        }
    }
}