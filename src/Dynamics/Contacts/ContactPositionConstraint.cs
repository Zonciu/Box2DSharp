using System.Numerics;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Contacts
{
    public class ContactPositionConstraint
    {
        public readonly Vector2[] LocalPoints = new Vector2[Settings.MaxManifoldPoints];

        public int IndexA;

        public int IndexB;

        public float InvIa, InvIb;

        public float InvMassA, InvMassB;

        public Vector2 LocalCenterA, LocalCenterB;

        public Vector2 LocalNormal;

        public Vector2 LocalPoint;

        public int PointCount;

        public float RadiusA, RadiusB;

        public ManifoldType Type;
    }
}