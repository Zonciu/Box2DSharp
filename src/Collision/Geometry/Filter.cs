namespace Box2DSharp
{
    /// This is used to filter collision on shapes. It affects shape-vs-shape collision
    ///	and shape-versus-query collision (such as b2World_CastRay).
    /// @ingroup shape
    public struct Filter
    {
        /// The collision category bits. Normally you would just set one bit. The category bits should
        ///	represent your application object types. For example:
        ///	@code{.cpp}
        ///	enum MyCategories
        ///	{
        ///	   Static  = 0x00000001,
        ///	   Dynamic = 0x00000002,
        ///	   Debris  = 0x00000004,
        ///	   Player  = 0x00000008,
        ///	   // etc
        /// };
        ///	@endcode
        public ulong CategoryBits;

        /// The collision mask bits. This states the categories that this
        /// shape would accept for collision.
        ///	For example, you may want your player to only collide with static objects
        ///	and other players.
        ///	@code{.c}
        ///	maskBits = Static | Player;
        ///	@endcode
        public ulong MaskBits;

        /// Collision groups allow a certain group of objects to never collide (negative)
        /// or always collide (positive). A group index of zero has no effect. Non-zero group filtering
        /// always wins against the mask bits.
        ///	For example, you may want ragdolls to collide with other ragdolls but you don't want
        ///	ragdoll self-collision. In this case you would give each ragdoll a unique negative group index
        ///	and apply that group index to all shapes on the ragdoll.
        public int GroupIndex;

        public Filter(ulong categoryBits, ulong maskBits, int groupIndex)
        {
            CategoryBits = categoryBits;
            MaskBits = maskBits;
            GroupIndex = groupIndex;
        }

        /// Use this to initialize your filter
        /// @ingroup shape
        public static Filter DefaultFilter()
        {
            Filter filter = new(0x0001UL, ulong.MaxValue, 0);
            return filter;
        }
    }
}