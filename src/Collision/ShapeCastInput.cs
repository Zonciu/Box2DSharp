using System;
using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Collision
{
    /// Input parameters for b2ShapeCast
    public struct ShapeCastInput
    {
        public DistanceProxy proxyA;

        public DistanceProxy proxyB;

        public Transform transformA;

        public Transform transformB;

        public Vector2 translationB;

      
    };
}