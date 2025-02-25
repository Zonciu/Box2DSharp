using System.Runtime.InteropServices;

namespace Box2DSharp
{
    [StructLayout(LayoutKind.Explicit, Pack = 4, Size = 32)]
    public struct BodyState
    {
        [FieldOffset(0)]
        public Vec2 LinearVelocity; // 8

        [FieldOffset(8)]
        public float AngularVelocity; // 4

        [FieldOffset(12)]
        public int Flags; // 4

        // Using delta position reduces round-off error far from the origin
        [FieldOffset(16)]
        public Vec2 DeltaPosition; // 8

        // Using delta rotation because I cannot access the full rotation on static bodies in
        // the solver and must use zero delta rotation for static bodies (c,s) = (1,0)
        [FieldOffset(24)]
        public Rot DeltaRotation; // 8

        public BodyState(Vec2 linearVelocity, float angularVelocity, int flags, Vec2 deltaPosition, Rot deltaRotation)
        {
            LinearVelocity = linearVelocity;
            AngularVelocity = angularVelocity;
            Flags = flags;
            DeltaPosition = deltaPosition;
            DeltaRotation = deltaRotation;
        }

        // Identity body state, notice the deltaRotation is {1, 0}
        public static BodyState Null;

        public static BodyState Identity = new(new(0.0f, 0.0f), 0.0f, 0, new(0.0f, 0.0f), new(1.0f, 0.0f));

        public void Reset()
        {
            LinearVelocity = default;
            AngularVelocity = default;
            Flags = default;
            DeltaPosition = default;
            DeltaRotation = default;
        }
    }
}