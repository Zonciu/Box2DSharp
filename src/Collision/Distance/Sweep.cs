namespace Box2DSharp
{
    /// <summary>
    /// This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to the body origin,
    /// which may not coincide with the center of mass. However, to support dynamics we must interpolate the center of mass
    /// position.
    /// </summary>
    public struct Sweep
    {
        public Vec2 LocalCenter;

        /// Local center of mass position
        public Vec2 C1;

        /// Starting center of mass world position
        public Vec2 C2;

        /// Ending center of mass world position
        public Rot Q1;

        /// Starting world rotation
        public Rot Q2; // Ending world rotation

        public Sweep(Vec2 localCenter, Vec2 c1, Vec2 c2, Rot q1, Rot q2)
        {
            LocalCenter = localCenter;
            C1 = c1;
            C2 = c2;
            Q1 = q1;
            Q2 = q2;
        }

        public static implicit operator Sweep((Vec2 localCenter, Vec2 c1, Vec2 c2, Rot q1, Rot q2) tuple)
        {
            return new Sweep(tuple.localCenter, tuple.c1, tuple.c2, tuple.q1, tuple.q2);
        }
    }
}