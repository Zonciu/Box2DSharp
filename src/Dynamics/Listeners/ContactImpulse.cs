using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Listeners
{
    /// Contact impulses for reporting. Impulses are used instead of forces because
    /// sub-step forces may approach infinity for rigid body collisions. These
    /// match up one-to-one with the contact points in b2Manifold.
    public struct ContactImpulse
    {
        public float[] NormalImpulses;

        public float[] TangentImpulses;

        public int Count;

        public static ContactImpulse Create()
        {
            return new ContactImpulse
            {
                Count           = 0,
                NormalImpulses  = new float[Settings.MaxManifoldPoints],
                TangentImpulses = new float[Settings.MaxManifoldPoints]
            };
        }
    }
}