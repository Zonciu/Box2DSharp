namespace Box2DSharp
{
    /// <summary>
    /// 柔软？
    /// </summary>
    public struct Softness
    {
        /// <summary>
        /// 偏差率
        /// </summary>
        public float BiasRate;

        /// <summary>
        /// 质量比例
        /// </summary>
        public float MassScale;

        /// <summary>
        /// 冲量比例
        /// </summary>
        public float ImpulseScale;

        public Softness(float biasRate, float massScale, float impulseScale)
        {
            BiasRate = biasRate;
            MassScale = massScale;
            ImpulseScale = impulseScale;
        }

        public static Softness MakeSoft(float hertz, float zeta, float h)
        {
            if (hertz == 0.0f)
            {
                return new Softness(0.0f, 1.0f, 0.0f);
            }

            float omega = 2.0f * B2Math.Pi * hertz;
            float a1 = 2.0f * zeta + h * omega;
            float a2 = h * omega * a1;
            float a3 = 1.0f / (1.0f + a2);
            return new Softness(omega / a1, a2 * a3, a3);
        }
    }
}