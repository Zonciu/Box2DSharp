using System;
using Box2DSharp;

namespace Testbed.Abstractions
{
    public class Camera
    {
        public Vec2 Center;

        public int Height;

        public int Width;

        public float Zoom;

        public Camera()
        {
            Width = 1280;
            Height = 800;
            ResetView();
        }

        public void ResetView()
        {
            Center.Set(0.0f, 0.0f);
            Zoom = 1.0f;
        }

        public Vec2 ConvertScreenToWorld(Vec2 screenPoint)
        {
            float w = Width;
            float h = Height;
            var u = screenPoint.X / w;
            var v = (h - screenPoint.Y) / h;

            var ratio = w / h;
            var extents = new Vec2(Zoom * ratio, Zoom);
            var lower = Center - extents;
            var upper = Center + extents;

            Vec2 pw;
            pw.X = (1.0f - u) * lower.X + u * upper.X;
            pw.Y = (1.0f - v) * lower.Y + v * upper.Y;
            return pw;
        }

        public Vec2 ConvertWorldToScreen(Vec2 worldPoint)
        {
            float w = Width;
            float h = Height;
            var ratio = w / h;
            var extents = new Vec2(Zoom * ratio, Zoom);

            var lower = Center - extents;
            var upper = Center + extents;

            var u = (worldPoint.X - lower.X) / (upper.X - lower.X);
            var v = (worldPoint.Y - lower.Y) / (upper.Y - lower.Y);

            var ps = new Vec2(u * w, (1.0f - v) * h);
            return ps;
        }

        public void BuildProjectionMatrix(Span<float> m, float zBias)
        {
            float ratio = Width / (float)Height;
            var extents = (Zoom * ratio, Zoom);

            var lower = Center - extents;
            var upper = Center + extents;
            float w = upper.X - lower.X;
            float h = upper.Y - lower.Y;

            m[0] = 2.0f / w;
            m[1] = 0.0f;
            m[2] = 0.0f;
            m[3] = 0.0f;

            m[4] = 0.0f;
            m[5] = 2.0f / h;
            m[6] = 0.0f;
            m[7] = 0.0f;

            m[8] = 0.0f;
            m[9] = 0.0f;
            m[10] = -1.0f;
            m[11] = 0.0f;

            m[12] = -2.0f * Center.X / w;
            m[13] = -2.0f * Center.Y / h;
            m[14] = zBias;
            m[15] = 1.0f;
        }

        public AABB GetViewBounds()
        {
            AABB bounds = new(ConvertScreenToWorld(new(0.0f, Height)), ConvertScreenToWorld(new(Width, 0.0f)));
            if (!bounds.IsValid)
            {
                return new AABB();
            }

            return bounds;
        }
    }
}