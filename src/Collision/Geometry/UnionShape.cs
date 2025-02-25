using System.Runtime.InteropServices;

namespace Box2DSharp
{
    [StructLayout(LayoutKind.Explicit)]
    public struct UnionShape
    {
        [FieldOffset(0)]
        public Capsule Capsule;

        [FieldOffset(0)]
        public Circle Circle;

        [FieldOffset(0)]
        public Polygon Polygon;

        [FieldOffset(0)]
        public Segment Segment;

        [FieldOffset(0)]
        public ChainSegment ChainSegment;
    }
}