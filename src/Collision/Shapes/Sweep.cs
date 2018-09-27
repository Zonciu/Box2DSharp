using System;
using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Collision.Shapes
{
    /// This describes the motion of a body/shape for TOI computation.
    /// Shapes are defined with respect to the body origin, which may
    /// no coincide with the center of mass. However, to support dynamics
    /// we must interpolate the center of mass position.
    public struct Sweep
    {
        /// <summary>
        /// Get the interpolated transform at a specific time.
        /// @param beta is a factor in [0,1], where 0 indicates alpha0.
        /// 获取特定时间的位置插值
        /// </summary>
        /// <param name="xf">位置</param>
        /// <param name="beta"></param>
        public void GetTransform(out Transform xf, float beta)
        {
            xf = new Transform();

            xf.Position = (1.0f - beta) * c0 + beta * c;
            var angle = (1.0f - beta) * a0 + beta * a;
            xf.Rotation.Set(angle);

            // Shift to origin
            xf.Position -= MathUtils.Mul(xf.Rotation, localCenter);
        }

        /// Advance the sweep forward, yielding a new initial state.
        /// @param alpha the new initial time.
        public void Advance(float alpha)
        {
            Debug.Assert(alpha0 < 1.0f);
            var beta = (alpha - alpha0) / (1.0f - alpha0);
            c0     += beta * (c - c0);
            a0     += beta * (a - a0);
            alpha0 =  alpha;
        }

        /// Normalize the angles.
        public void Normalize()
        {
            var twoPi = 2.0f * Settings.Pi;
            var d     = twoPi * (float) Math.Floor(a0 / twoPi);
            a0 -= d;
            a  -= d;
        }

        /// <summary>
        /// local center of mass position
        /// </summary>
        public Vector2 localCenter;

        /// <summary>
        /// center world positions
        /// </summary>
        public Vector2 c0, c;

        /// <summary>
        /// world angles
        /// </summary>
        public float a0, a;

        /// Fraction of the current time step in the range [0,1]
        /// c0 and a0 are the positions at alpha0.
        public float alpha0;
    };
}