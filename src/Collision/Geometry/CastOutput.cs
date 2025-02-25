namespace Box2DSharp
{
    /// <summary>
    ///  Low level ray-cast or shape-cast output data
    /// </summary>
    public struct CastOutput
    {
        /// <summary>
        ///  The surface normal at the hit point
        /// </summary>
        public Vec2 Normal;

        /// <summary>
        ///  The surface hit point
        /// </summary>
        public Vec2 Point;

        /// <summary>
        ///  The fraction of the input translation at collision
        /// </summary>
        public float Fraction;

        /// <summary>
        ///  The number of iterations used
        /// </summary>
        public int Iterations;

        /// <summary>
        ///  Did the cast hit?
        /// </summary>
        public bool Hit;
    }
}