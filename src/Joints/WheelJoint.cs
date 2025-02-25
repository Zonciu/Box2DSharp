namespace Box2DSharp
{
    public class WheelJoint
    {
        public Vec2 LocalAxisA;

        public float PerpImpulse;

        public float MotorImpulse;

        public float SpringImpulse;

        public float LowerImpulse;

        public float UpperImpulse;

        public float MaxMotorTorque;

        public float MotorSpeed;

        public float LowerTranslation;

        public float UpperTranslation;

        public float Hertz;

        public float DampingRatio;

        public int IndexA;

        public int IndexB;

        public Vec2 AnchorA;

        public Vec2 AnchorB;

        public Vec2 AxisA;

        public Vec2 DeltaCenter;

        public float PerpMass;

        public float MotorMass;

        public float AxialMass;

        public Softness SpringSoftness;

        public bool EnableSpring;

        public bool EnableMotor;

        public bool EnableLimit;
    }
}