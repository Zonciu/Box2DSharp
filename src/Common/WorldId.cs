using System;

namespace Box2DSharp
{
    /// <summary>
    /// World id references a world instance. This should be treated as an opaque handle.
    /// </summary>
    public struct WorldId : IEquatable<WorldId>
    {
        public ushort Index1;

        public ushort Revision;

        public static WorldId NullId = new();

        public bool IsNull => Index1 == 0;

        public bool IsNotNull => Index1 != 0;

        public WorldId(ushort index1, ushort revision)
        {
            Index1 = index1;
            Revision = revision;
        }

        public bool Equals(WorldId other)
        {
            return Index1 == other.Index1 && Revision == other.Revision;
        }

        public override bool Equals(object? obj)
        {
            return obj is WorldId other && Equals(other);
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(Index1, Revision);
        }

        public static bool operator ==(WorldId left, WorldId right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(WorldId left, WorldId right)
        {
            return !(left == right);
        }
    }
}