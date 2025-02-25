namespace Box2DSharp
{
    /// <summary>
    /// Simplex vertex for debugging the GJK algorithm
    /// </summary>
    public struct SimplexVertex
    {
        public Vec2 WA;

        /// support point in proxyA
        public Vec2 WB;

        /// support point in proxyB
        public Vec2 W;

        /// wB - wA
        public float A;

        /// barycentric coordinate for closest point
        public int IndexA;

        /// wA index
        public int IndexB; // wB index
    }
}