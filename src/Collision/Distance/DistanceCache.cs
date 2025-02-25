namespace Box2DSharp
{
    /// <summary>
    /// Used to warm start b2Distance. Set count to zero on first call or
    /// </summary>
    ///	use zero initialization.
    public struct DistanceCache
    {
        /// <summary>
        /// The number of stored simplex points
        /// </summary>
        public ushort Count;

        /// <summary>
        /// The cached simplex indices on shape A
        /// </summary>
        public FixedArray3<byte> IndexA;

        /// <summary>
        /// The cached simplex indices on shape B
        /// </summary>
        public FixedArray3<byte> IndexB;
    }
}