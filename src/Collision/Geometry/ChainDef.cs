namespace Box2DSharp
{
    /// Used to create a chain of line segments. This is designed to eliminate ghost collisions with some limitations.
    ///	- chains are one-sided
    ///	- chains have no mass and should be used on static bodies
    ///	- chains have a counter-clockwise winding order
    ///	- chains are either a loop or open
    /// - a chain must have at least 4 points
    ///	- the distance between any two points must be greater than b2_linearSlop
    ///	- a chain shape should not self intersect (this is not validated)
    ///	- an open chain shape has NO COLLISION on the first and final edge
    ///	- you may overlap two open chains on their first three and/or last three points to get smooth collision
    ///	- a chain shape creates multiple line segment shapes on the body
    /// https://en.wikipedia.org/wiki/Polygonal_chain
    /// Must be initialized using b2DefaultChainDef().
    ///	@warning Do not use chain shapes unless you understand the limitations. This is an advanced feature.
    /// @ingroup shape
    public struct ChainDef
    {
        /// Use this to store application specific shape data.
        public object UserData;

        /// An array of at least 4 points. These are cloned and may be temporary.
        public Vec2[] Points;

        /// The point count, must be 4 or more.
        public int Count;

        /// The friction coefficient, usually in the range [0,1].
        public float Friction;

        /// The restitution (elasticity) usually in the range [0,1].
        public float Restitution;

        /// Contact filtering data.
        public Filter Filter;

        /// Indicates a closed chain formed by connecting the first and last points
        public bool IsLoop;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int InternalValue;

        /// Use this to initialize your chain definition
        /// @ingroup shape
        public static ChainDef DefaultChainDef()
        {
            return new ChainDef
            {
                Friction = 0.6f,
                Filter = Filter.DefaultFilter(),
                InternalValue = Core.SecretCookie
            };
        }
    }
}