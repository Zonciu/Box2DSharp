using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Listeners
{
    /// Contact impulses for reporting. Impulses are used instead of forces because
    /// sub-step forces may approach infinity for rigid body collisions. These
    /// match up one-to-one with the contact points in b2Manifold.
    public struct ContactImpulse
    {
        public float[] normalImpulses;

        public float[] tangentImpulses;

        public int count;

        public static ContactImpulse Create()
        {
            return new ContactImpulse
            {
                count           = 0,
                normalImpulses  = new float[Settings.MaxManifoldPoints],
                tangentImpulses = new float[Settings.MaxManifoldPoints]
            };
        }
    };
}