namespace Box2DSharp
{
    /// <summary>
    ///  A manifold point is a contact point belonging to a contact
    ///  manifold. It holds details related to the geometry and dynamics
    ///  of the contact points.
    /// </summary>
    public struct ManifoldPoint
    {
        /// <summary>
        ///  Location of the contact point in world space. Subject to precision loss at large coordinates.
        /// @note Should only be used for debugging.
        /// </summary>
        public Vec2 Point;

        /// <summary>
        ///  Location of the contact point relative to bodyA's origin in world space
        /// </summary>
        /// <summary>
        ///  @note When used internally to the Box2D solver, these are relative to the center of mass.
        /// </summary>
        public Vec2 AnchorA;

        /// <summary>
        ///  Location of the contact point relative to bodyB's origin in world space
        /// </summary>
        public Vec2 AnchorB;

        /// <summary>
        ///  The separation of the contact point, negative if penetrating
        /// </summary>
        public float Separation;

        /// <summary>
        ///  The impulse along the manifold normal vector.
        /// </summary>
        public float NormalImpulse;

        /// <summary>
        ///  The friction impulse
        /// </summary>
        public float TangentImpulse;

        /// <summary>
        ///  The maximum normal impulse applied during sub-stepping
        /// </summary>
        /// <summary>
        ///  todo not sure this is needed
        /// </summary>
        public float MaxNormalImpulse;

        /// <summary>
        ///  Relative normal velocity pre-solve. Used for hit events. If the normal impulse is
        /// </summary>
        /// <summary>
        ///  zero then there was no hit. Negative means shapes are approaching.
        /// </summary>
        public float NormalVelocity;

        /// <summary>
        ///  Uniquely identifies a contact point between two shapes
        /// </summary>
        public ushort Id;

        /// <summary>
        ///  Did this contact point exist the previous step?
        /// </summary>
        public bool Persisted;
    }
}