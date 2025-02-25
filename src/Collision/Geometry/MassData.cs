namespace Box2DSharp
{
    /// <summary>
    ///  This holds the mass data computed for a shape.
    /// </summary>
    public struct MassData
    {
        /// <summary>
        /// The mass of the shape, usually in kilograms.
        /// </summary>
        public float Mass;

        /// <summary>
        /// The position of the shape's centroid relative to the shape's origin.
        /// </summary>
        public Vec2 Center;

        /// <summary>
        /// The rotational inertia of the shape about the local origin.
        /// </summary>
        public float RotationalInertia;

        public MassData(float mass, Vec2 center, float rotationalInertia)
        {
            Mass = mass;
            Center = center;
            RotationalInertia = rotationalInertia;
        }

        public static implicit operator MassData((float mass, Vec2 center, float rotationalInertia) tuple)
        {
            return new MassData(tuple.mass, tuple.center, tuple.rotationalInertia);
        }
    }
}