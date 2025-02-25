namespace Box2DSharp
{
    /// <summary>
    /// Input parameters for b2TimeOfImpact
    /// </summary>
    public struct TOIInput
    {
        public DistanceProxy ProxyA;

        /// The proxy for shape A
        public DistanceProxy ProxyB;

        /// The proxy for shape B
        public Sweep SweepA;

        /// The movement of shape A
        public Sweep SweepB;

        /// The movement of shape B
        public float TMax; // Defines the sweep interval [0, tMax]
    }
}