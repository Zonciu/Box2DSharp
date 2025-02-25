namespace Box2DSharp
{
    /// <summary>
    /// Output for b2ShapeDistance
    /// </summary>
    public struct DistanceOutput
    {
        public Vec2 PointA;

        /// Closest point on shapeA
        public Vec2 PointB;

        /// Closest point on shapeB
        public float Distance;

        /// The final distance, zero if overlapped
        public int Iterations;

        /// Number of GJK iterations used
        public int SimplexCount; // The number of simplexes stored in the simplex array
    }
}