namespace Box2DSharp
{
    public class RevoluteJoint
    {
        public Vec2 LinearImpulse;

        public float SpringImpulse;

        public float MotorImpulse;

        public float LowerImpulse;

        public float UpperImpulse;

        public float Hertz;

        public float DampingRatio;

        public float MaxMotorTorque;

        public float MotorSpeed;

        public float ReferenceAngle;

        public float LowerAngle;

        public float UpperAngle;

        public int IndexA;

        public int IndexB;

        public Vec2 AnchorA;

        public Vec2 AnchorB;

        public Vec2 DeltaCenter;

        public float DeltaAngle;

        public float AxialMass;

        public Softness SpringSoftness;

        public bool EnableSpring;

        public bool EnableMotor;

        public bool EnableLimit;
    }
}