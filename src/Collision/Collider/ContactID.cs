using System.Runtime.InteropServices;

namespace Box2DSharp.Collision.Collider
{
    /// Contact ids to facilitate warm starting.
    [StructLayout(LayoutKind.Explicit, Size = 4)]
    public struct ContactID
    {
        [FieldOffset(0)]
        public ContactFeature cf;

        /// Used to quickly compare contact ids.
        [FieldOffset(0)]
        public uint key;
    };
}