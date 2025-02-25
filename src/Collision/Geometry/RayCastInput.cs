namespace Box2DSharp
{
    /// <summary>
    /// Low level ray-cast input data
    /// </summary>
    public struct RayCastInput
    {
        /// <summary>
        /// Start point of the ray cast
        /// </summary>
        public Vec2 Origin;

        /// <summary>
        /// Translation of the ray cast
        /// </summary>
        public Vec2 Translation;

        /// <summary>
        /// The maximum fraction of the translation to consider, typically 1
        /// </summary>
        public float MaxFraction;

        public RayCastInput(Vec2 origin, Vec2 translation, float maxFraction)
        {
            Origin = origin;
            Translation = translation;
            MaxFraction = maxFraction;
        }

        public static implicit operator RayCastInput((Vec2 origin, Vec2 translation, float maxFraction) tuple)
        {
            return new RayCastInput(tuple.origin, tuple.translation, tuple.maxFraction);
        }
    }
}