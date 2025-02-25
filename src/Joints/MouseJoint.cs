namespace Box2DSharp
{
    public class MouseJoint
    {
        public Vec2 TargetA;

        public float Hertz;

        public float DampingRatio;

        public float MaxForce;

        public Vec2 LinearImpulse;

        public float AngularImpulse;

        public Softness LinearSoftness;

        public Softness AngularSoftness;

        public int IndexB;

        public Vec2 AnchorB;

        public Vec2 DeltaCenter;

        public Mat22 LinearMass;
    }
}