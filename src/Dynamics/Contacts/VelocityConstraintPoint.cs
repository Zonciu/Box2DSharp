using System.Numerics;

namespace Box2DSharp.Dynamics.Contacts
{
    public class VelocityConstraintPoint
    {
        public Vector2 Ra;

        public Vector2 Rb;

        public float NormalImpulse;

        public float TangentImpulse;

        public float NormalMass;

        public float TangentMass;

        public float VelocityBias;
    };
}