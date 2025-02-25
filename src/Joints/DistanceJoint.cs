namespace Box2DSharp
{
    public class DistanceJoint
    {
        public float Length;

        public float Hertz;

        public float DampingRatio;

        public float MinLength;

        public float MaxLength;

        public float MaxMotorForce;

        public float MotorSpeed;

        public float Impulse;

        public float LowerImpulse;

        public float UpperImpulse;

        public float MotorImpulse;

        public int IndexA;

        public int IndexB;

        public Vec2 AnchorA;

        public Vec2 AnchorB;

        public Vec2 DeltaCenter;

        public Softness DistanceSoftness;

        public float AxialMass;

        public bool EnableSpring;

        public bool EnableLimit;

        public bool EnableMotor;
    }
}