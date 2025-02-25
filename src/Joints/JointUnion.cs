using System.Runtime.InteropServices;

namespace Box2DSharp
{
    [StructLayout(LayoutKind.Explicit)]
    public struct JointUnion
    {
        [FieldOffset(0)]
        public DistanceJoint DistanceJoint;

        [FieldOffset(0)]
        public MotorJoint MotorJoint;

        [FieldOffset(0)]
        public MouseJoint MouseJoint;

        [FieldOffset(0)]
        public RevoluteJoint RevoluteJoint;

        [FieldOffset(0)]
        public PrismaticJoint PrismaticJoint;

        [FieldOffset(0)]
        public WeldJoint WeldJoint;

        [FieldOffset(0)]
        public WheelJoint WheelJoint;
    }
}