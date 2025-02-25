namespace Box2DSharp
{
    /// <summary>
    /// Low level shape cast input in generic form. This allows casting an arbitrary point
    ///	cloud wrap with a radius. For example, a circle is a single point with a non-zero radius.
    ///	A capsule is two points with a non-zero radius. A box is four points with a zero radius.
    /// </summary>
    public struct ShapeCastInput
    {
        /// <summary>
        ///  A point cloud to cast
        /// </summary>
        public FixedArray8<Vec2> Points;

        /// <summary>
        ///  The number of points
        /// </summary>
        public int Count;

        /// <summary>
        ///  The radius around the point cloud
        /// </summary>
        public float Radius;

        /// <summary>
        ///  The translation of the shape cast
        /// </summary>
        public Vec2 Translation;

        /// <summary>
        ///  The maximum fraction of the translation to consider, typically 1
        /// </summary>
        public float MaxFraction;
    }
}