namespace Box2DSharp
{
    public class MotorJoint
    {
        public Vec2 LinearOffset;

        public float AngularOffset;

        public Vec2 LinearImpulse;

        public float AngularImpulse;

        public float MaxForce;

        public float MaxTorque;

        public float CorrectionFactor;

        public int IndexA;

        public int IndexB;

        public Vec2 AnchorA;

        public Vec2 AnchorB;

        public Vec2 DeltaCenter;

        public float DeltaAngle;

        public Mat22 LinearMass;

        public float AngularMass;
    }
}