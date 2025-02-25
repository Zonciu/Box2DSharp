namespace Box2DSharp
{
    public class WeldJoint
    {
        public float ReferenceAngle;

        public float LinearHertz;

        public float LinearDampingRatio;

        public float AngularHertz;

        public float AngularDampingRatio;

        public Softness LinearSoftness;

        public Softness AngularSoftness;

        public Vec2 LinearImpulse;

        public float AngularImpulse;

        public int IndexA;

        public int IndexB;

        public Vec2 AnchorA;

        public Vec2 AnchorB;

        public Vec2 DeltaCenter;

        public float DeltaAngle;

        public float AxialMass;
    }
}