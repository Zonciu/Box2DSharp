namespace Box2DSharp
{
    /// <summary>
    /// A distance proxy is used by the GJK algorithm. It encapsulates any shape.
    /// </summary>
    public struct DistanceProxy
    {
        /// <summary>
        /// The point cloud
        /// </summary>
        public FixedArray8<Vec2> Points;

        /// <summary>
        /// The number of points
        /// </summary>
        public int Count;

        /// <summary>
        /// The external radius of the point cloud
        /// </summary>
        public float Radius;
    }
}