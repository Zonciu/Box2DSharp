using System;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace Box2DSharp
{
    /// <summary>
    /// Body id references a body instance. This should be treated as an opaque handle.
    /// </summary>
    [StructLayout(LayoutKind.Explicit)]
    public struct BodyId : IEquatable<BodyId>
    {
        [FieldOffset(0)]
        public readonly ulong Value;

        [FieldOffset(0)]
        public readonly int Index1;

        [FieldOffset(4)]
        public readonly ushort World0;

        [FieldOffset(6)]
        public readonly ushort Revision;

        public static BodyId NullId = new();

        public bool IsNull => Index1 == 0;

        public bool IsNotNull => Index1 != 0;

        public BodyId(ulong value)
        {
            Index1 = 0;
            World0 = 0;
            Revision = 0;
            Value = value;
        }

        public BodyId(int index1, ushort world0, ushort revision)
        {
            Value = 0;
            Index1 = index1;
            World0 = world0;
            Revision = revision;
        }

        public readonly bool IsValid()
        {
            if (Core.MaxWorlds <= World0)
            {
                // invalid world
                return false;
            }

            var world = World.Worlds[World0];
            if (world!.WorldId != World0)
            {
                // world is free
                return false;
            }

            if (Index1 < 1 || world.BodyArray.Count < Index1)
            {
                // invalid index
                return false;
            }

            Body body = world.BodyArray[Index1 - 1];
            if (body.SetIndex == Core.NullIndex)
            {
                // this was freed
                return false;
            }

            Debug.Assert(body.LocalIndex != Core.NullIndex);

            if (body.Revision != Revision)
            {
                // this id is orphaned
                return false;
            }

            return true;
        }

        public bool Equals(BodyId other)
        {
            return Index1 == other.Index1
                && World0 == other.World0
                && Revision == other.Revision;
        }

        public override bool Equals(object? obj)
        {
            return obj is BodyId other && Equals(other);
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(Index1, World0, Revision);
        }

        public static bool operator ==(BodyId left, BodyId right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(BodyId left, BodyId right)
        {
            return !(left == right);
        }
    }
}