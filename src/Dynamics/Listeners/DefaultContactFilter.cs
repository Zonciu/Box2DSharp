namespace Box2DSharp.Dynamics.Listeners
{
    /// Implement this class to provide collision filtering. In other words, you can implement
    /// this class if you want finer control over contact creation.
    public sealed class DefaultContactFilter : IContactFilter
    {
        /// Return true if contact calculations should be performed between these two shapes.
        /// @warning for performance reasons this is only called when the AABBs begin to overlap.
        public bool ShouldCollide(Fixture fixtureA, Fixture fixtureB)
        {
            ref readonly var filterA = ref fixtureA.GetFilterData();
            ref readonly var filterB = ref fixtureB.GetFilterData();

            if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0)
            {
                return filterA.groupIndex > 0;
            }

            var collide = (filterA.maskBits & filterB.categoryBits) != 0
                       && (filterA.categoryBits & filterB.maskBits) != 0;
            return collide;
        }
    }
}