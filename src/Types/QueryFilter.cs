namespace Box2DSharp
{
    /// The query filter is used to filter collisions between queries and shapes. For example,
    ///	you may want a ray-cast representing a projectile to hit players and the static environment
    ///	but not debris.
    /// @ingroup shape
    public struct QueryFilter
    {
        /// The collision category bits of this query. Normally you would just set one bit.
        public ulong CategoryBits;

        /// The collision mask bits. This states the shape categories that this
        /// query would accept for collision.
        public ulong MaskBits;

        public QueryFilter(ulong categoryBits, ulong maskBits)
        {
            CategoryBits = categoryBits;
            MaskBits = maskBits;
        }

        /// Use this to initialize your query filter
        /// @ingroup shape
        public static QueryFilter DefaultQueryFilter()
        {
            QueryFilter filter = new(0x0001UL, ulong.MaxValue);
            return filter;
        }
    }
}