namespace Box2DSharp
{
    /// <summary>
    /// A 2-by-2 Matrix (16 bytes)
    /// </summary>
    public struct Mat22
    {
        /// <summary>
        /// columns
        /// </summary>
        public Vec2 Cx;

        /// <summary>
        /// columns
        /// </summary>
        public Vec2 Cy;

        public Mat22(Vec2 cx, Vec2 cy)
        {
            Cx = cx;
            Cy = cy;
        }

        public static Mat22 Zero = new(new(0, 0), new(0, 0));
    }
}