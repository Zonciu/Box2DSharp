using System;
using System.Numerics;

namespace Box2DSharp.Common
{
    /// Rotation
    public struct Rotation
    {
        /// Initialize from an angle in radians
        public Rotation(float angle)
        {
            // TODO_ERIN optimize
            Sin = (float) Math.Sin(angle);
            Cos = (float) Math.Cos(angle);
        }

        /// Set using an angle in radians.
        public void Set(float angle)
        {
            // TODO_ERIN optimize
            Sin = (float) Math.Sin(angle);
            Cos = (float) Math.Cos(angle);
        }

        /// Set to the identity rotation
        public void SetIdentity()
        {
            Sin = 0.0f;
            Cos = 1.0f;
        }

        /// Get the angle in radians
        public float GetAngle => (float) Math.Atan2(Sin, Cos);

        /// Get the x-axis
        public Vector2 GetXAxis() => new Vector2(Cos, Sin);

        /// Get the u-axis
        public Vector2 GetYAxis() => new Vector2(-Sin, Cos);

        /// Sine and cosine
        public float Sin;

        public float Cos;
    };
}