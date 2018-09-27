using System.Numerics;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Contacts
{
    public class ContactPositionConstraint
    {
        public Vector2[] localPoints = new Vector2[Settings.MaxManifoldPoints];

        public Vector2 localNormal;

        public Vector2 localPoint;

        public int indexA;

        public int indexB;

        public float invMassA, invMassB;

        public Vector2 localCenterA, localCenterB;

        public float invIA, invIB;

        public ManifoldType type;

        public float radiusA, radiusB;

        public int pointCount;
    };
}