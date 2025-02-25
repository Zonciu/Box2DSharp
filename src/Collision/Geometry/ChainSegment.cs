namespace Box2DSharp
{
    /// <summary>
    /// A line segment with one-sided collision(36 bytes). Only collides on the right side.
    /// Several of these are generated for a chain shape.
    /// ghost1 -> point1 -> point2 -> ghost2
    /// </summary>
    public struct ChainSegment
    {
        /// <summary>
        /// The tail ghost vertex
        /// </summary>
        public Vec2 Ghost1;

        /// <summary>
        /// The line segment
        /// </summary>
        public Segment Segment;

        /// <summary>
        /// The head ghost vertex
        /// </summary>
        public Vec2 Ghost2;

        /// <summary>
        /// The owning chain shape index (internal usage only)
        /// </summary>
        public int ChainId;

        public ChainSegment(Vec2 ghost1, Segment segment, Vec2 ghost2, int chainId)
        {
            Ghost1 = ghost1;
            Segment = segment;
            Ghost2 = ghost2;
            ChainId = chainId;
        }

        public static implicit operator ChainSegment((Vec2 Ghost1, Segment Segment, Vec2 Ghost2, int ChainId) tuple)
        {
            return new ChainSegment(tuple.Ghost1, tuple.Segment, tuple.Ghost2, tuple.ChainId);
        }
    }
}