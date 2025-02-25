namespace Box2DSharp
{
    /// <summary>
    /// Output parameters for b2TimeOfImpact.
    /// </summary>
    public struct TOIOutput
    {
        public TOIState State;

        /// The type of result
        public float T; // The time of the collision
    }
}