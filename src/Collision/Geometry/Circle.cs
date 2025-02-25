namespace Box2DSharp
{
    /// <summary>
    /// A solid circle (12 bytes)
    /// </summary>
    public struct Circle
    {
        /// <summary>
        /// The local center
        /// </summary>
        public Vec2 Center;

        /// <summary>
        /// The radius
        /// </summary>
        public float Radius;

        public Circle(Vec2 center, float radius)
        {
            Center = center;
            Radius = radius;
        }

        public static implicit operator Circle((Vec2 Center, float Radius) tuple)
        {
            return new Circle(tuple.Center, tuple.Radius);
        }
    }
}