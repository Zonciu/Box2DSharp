using System;

namespace Box2DSharp
{
    /// <summary>
    /// Chain id references a chain instances. This should be treated as an opaque handle.
    /// </summary>
    public struct ChainId : IEquatable<ChainId>
    {
        public int Index1;

        public ushort World0;

        public ushort Revision;

        public static ChainId NullId = new();

        public bool IsNull => Index1 == 0;

        public bool IsNotNull => Index1 != 0;

        public ChainId(int index1, ushort world0, ushort revision)
        {
            Index1 = index1;
            World0 = world0;
            Revision = revision;
        }

        public bool Equals(ChainId other)
        {
            return Index1 == other.Index1
                && World0 == other.World0
                && Revision == other.Revision;
        }

        public override bool Equals(object? obj)
        {
            return obj is ChainId other && Equals(other);
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(Index1, World0, Revision);
        }

        public static bool operator ==(ChainId left, ChainId right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(ChainId left, ChainId right)
        {
            return !(left == right);
        }
    }
}