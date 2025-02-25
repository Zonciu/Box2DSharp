using System;

namespace Box2DSharp
{
    /// <summary>
    /// Joint id references a joint instance. This should be treated as an opaque handle.
    /// </summary>
    public struct JointId : IEquatable<JointId>
    {
        public int Index1;

        public ushort World0;

        public ushort Revision;

        public static JointId NullId = new();

        public bool IsNull => Index1 == 0;

        public bool IsNotNull => Index1 != 0;

        public JointId(int index1, ushort world0, ushort revision)
        {
            Index1 = index1;
            World0 = world0;
            Revision = revision;
        }

        public bool Equals(JointId other)
        {
            return Index1 == other.Index1
                && World0 == other.World0
                && Revision == other.Revision;
        }

        public override bool Equals(object? obj)
        {
            return obj is JointId other && Equals(other);
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(Index1, World0, Revision);
        }

        public static bool operator ==(JointId left, JointId right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(JointId left, JointId right)
        {
            return !(left == right);
        }
    }
}