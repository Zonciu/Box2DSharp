using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Contacts
{
    public class ContactVelocityConstraint
    {
        public VelocityConstraintPoint[] points = new VelocityConstraintPoint[Settings.MaxManifoldPoints]
        {
            new VelocityConstraintPoint(),
            new VelocityConstraintPoint()
        };

        public Vector2 normal;

        public Matrix2x2 normalMass;

        public Matrix2x2 K;

        public int indexA;

        public int indexB;

        public float invMassA, invMassB;

        public float invIA, invIB;

        public float friction;

        public float restitution;

        public float tangentSpeed;

        public int pointCount;

        public int contactIndex;
    };
}