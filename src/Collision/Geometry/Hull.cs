namespace Box2DSharp
{
    /// <summary>
    /// A convex hull. Used to create convex polygons.
    ///	@warning Do not modify these values directly, instead use b2ComputeHull()
    /// </summary>
    public struct Hull
    {
        /// The final points of the hull
        public FixedArray8<Vec2> Points;

        /// The number of points
        public int Count;
    }
}