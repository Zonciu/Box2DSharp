namespace Box2DSharp
{
    public class PrismaticJoint
    {
        public Vec2 LocalAxisA;

        public Vec2 Impulse;

        public float SpringImpulse;

        public float MotorImpulse;

        public float LowerImpulse;

        public float UpperImpulse;

        public float Hertz;

        public float DampingRatio;

        public float MaxMotorForce;

        public float MotorSpeed;

        public float ReferenceAngle;

        public float LowerTranslation;

        public float UpperTranslation;

        public int IndexA;

        public int IndexB;

        public Vec2 AnchorA;

        public Vec2 AnchorB;

        public Vec2 AxisA;

        public Vec2 DeltaCenter;

        public float DeltaAngle;

        public float AxialMass;

        public Softness SpringSoftness;

        public bool EnableSpring;

        public bool EnableLimit;

        public bool EnableMotor;
    }
}