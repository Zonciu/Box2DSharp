namespace Box2DSharp
{
    /// The body simulation type.
    /// Each body is one of these three types. The type determines how the body behaves in the simulation.
    /// @ingroup body
    public enum BodyType
    {
        /// zero mass, zero velocity, may be manually moved
        StaticBody = 0,

        /// zero mass, velocity set by user, moved by solver
        KinematicBody = 1,

        /// positive mass, velocity determined by forces, moved by solver
        DynamicBody = 2,

        /// number of body types
        BodyTypeCount,
    }
}