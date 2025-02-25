using System;

namespace Box2DSharp
{
    /// <summary>
    /// Result from b2World_RayCastClosest
    /// </summary>
    public class RayResult : IDisposable
    {
        public ShapeId ShapeId;

        public Vec2 Point;

        public Vec2 Normal;

        public float Fraction;

        public bool Hit;

        private bool _disposed;

        public void Dispose()
        {
            if (_disposed)
            {
                return;
            }

            B2ObjectPool<RayResult>.Shared.Return(this);
            _disposed = true;
        }
    }
}