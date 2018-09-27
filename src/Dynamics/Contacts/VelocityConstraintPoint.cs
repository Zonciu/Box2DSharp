using System.Numerics;

namespace Box2DSharp.Dynamics.Contacts
{
    public class VelocityConstraintPoint
    {
        public Vector2 rA;

        public Vector2 rB;

        public float normalImpulse;

        public float tangentImpulse;

        public float normalMass;

        public float tangentMass;

        public float velocityBias;
    };
}