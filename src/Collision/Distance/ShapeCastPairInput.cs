namespace Box2DSharp
{
    /// <summary>
    /// Input parameters for b2ShapeCast
    /// </summary>
    public struct ShapeCastPairInput
    {
        public DistanceProxy ProxyA;

        /// The proxy for shape A
        public DistanceProxy ProxyB;

        /// The proxy for shape B
        public Transform TransformA;

        /// The world transform for shape A
        public Transform TransformB;

        /// The world transform for shape B
        public Vec2 TranslationB;

        /// The translation of shape B
        public float MaxFraction; // The fraction of the translation to consider, typically 1
    }
}