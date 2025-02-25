using System.Runtime.InteropServices;

namespace Box2DSharp
{
    /// <summary>
    /// 2D rotation (8 bytes)
    /// This is similar to using a complex number for rotation
    /// </summary>
    [StructLayout(LayoutKind.Explicit, Size = 8)]
    public struct Rot
    {
        /// <summary>
        /// cosine
        /// </summary>
        [FieldOffset(0)]
        public float C;

        /// <summary>
        /// sine
        /// </summary>
        [FieldOffset(4)]
        public float S;

        public Rot(float c, float s)
        {
            C = c;
            S = s;
        }

        public static implicit operator Rot((float c, float s) tuple)
        {
            return new Rot(tuple.c, tuple.s);
        }

        public static readonly Rot Identity = new(1, 0);

        public override string ToString()
        {
            return $"{C},{S}";
        }

        /// Get the angle in radians in the range [-pi, pi]
        public readonly float GetAngle()
        {
            return B2Math.Atan2(S, C);
        }

        /// Get the x-axis
        public readonly Vec2 GetXAxis()
        {
            Vec2 v = new(C, S);
            return v;
        }

        /// Get the y-axis
        public readonly Vec2 GetYAxis()
        {
            Vec2 v = new(-S, C);
            return v;
        }

        /// Is this rotation normalized?
        public readonly bool IsNormalized()
        {
            // larger tolerance due to failure on mingw 32-bit
            var qq = S * S + C * C;
            return qq is > 1.0f - 0.0006f and < 1.0f + 0.0006f;
        }
    }
}