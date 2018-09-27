using Box2DSharp.Common;

namespace Box2DSharp.Collision
{
    /// Input for b2Distance.
    /// You have to option to use the shape radii
    /// in the computation. Even 
    public struct DistanceInput
    {
        public DistanceProxy proxyA;

        public DistanceProxy proxyB;

        public Transform transformA;

        public Transform transformB;

        public bool useRadii;
    };
}