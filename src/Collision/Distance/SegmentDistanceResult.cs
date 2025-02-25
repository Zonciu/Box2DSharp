namespace Box2DSharp
{
    /// <summary>
    /// Result of computing the distance between two line segments
    /// </summary>
    public struct SegmentDistanceResult
    {
        /// <summary>
        /// The closest point on the first segment
        /// </summary>
        public Vec2 Closest1;

        /// <summary>
        /// The closest point on the second segment
        /// </summary>
        public Vec2 Closest2;

        /// <summary>
        /// The barycentric coordinate on the first segment
        /// </summary>
        public float Fraction1;

        /// <summary>
        /// The barycentric coordinate on the second segment
        /// </summary>
        public float Fraction2;

        /// <summary>
        /// The squared distance between the closest points
        /// </summary>
        public float DistanceSquared;
    }
}