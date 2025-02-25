namespace Box2DSharp
{
    /// <summary>
    /// Input for b2ShapeDistance
    /// </summary>
    public struct DistanceInput
    {
        /// <summary>
        /// The proxy for shape A
        /// </summary>
        public DistanceProxy ProxyA;

        /// <summary>
        /// The proxy for shape B
        /// </summary>
        public DistanceProxy ProxyB;

        /// <summary>
        /// The world transform for shape A
        /// </summary>
        public Transform TransformA;

        /// <summary>
        /// The world transform for shape B
        /// </summary>
        public Transform TransformB;

        /// <summary>
        /// Should the proxy radius be considered?
        /// </summary>
        public bool UseRadii;
    }
}