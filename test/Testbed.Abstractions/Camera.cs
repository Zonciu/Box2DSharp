using System.Numerics;
using Box2DSharp.Common;

namespace Testbed.Abstractions
{
    public class Camera
    {
        public Vector2 Center;

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
            Center.Set(0.0f, 20.0f);
            Zoom = 1.0f;
        }

        public Vector2 ConvertScreenToWorld(Vector2 screenPoint)
        {
            float w = Width;
            float h = Height;
            var u = screenPoint.X / w;
            var v = (h - screenPoint.Y) / h;

            var ratio = w / h;
            var extents = new Vector2(ratio * 25.0f, 25.0f);
            extents *= Zoom;

            var lower = Center - extents;
            var upper = Center + extents;

            Vector2 pw;
            pw.X = (1.0f - u) * lower.X + u * upper.X;
            pw.Y = (1.0f - v) * lower.Y + v * upper.Y;
            return pw;
        }

        public Vector2 ConvertWorldToScreen(Vector2 worldPoint)
        {
            float w = Width;
            float h = Height;
            var ratio = w / h;
            var extents = new Vector2(ratio * 25.0f, 25.0f);
            extents *= Zoom;

            var lower = Center - extents;
            var upper = Center + extents;

            var u = (worldPoint.X - lower.X) / (upper.X - lower.X);
            var v = (worldPoint.Y - lower.Y) / (upper.Y - lower.Y);

            var ps = new Vector2(u * w, (1.0f - v) * h);
            return ps;
        }

        public void BuildProjectionMatrix(float[] m, float zBias)
        {
            float w = Width;
            float h = Height;
            var ratio = w / h;
            var extents = new Vector2(ratio * 25.0f, 25.0f);
            extents *= Zoom;

            var lower = Center - extents;
            var upper = Center + extents;

            m[0] = 2.0f / (upper.X - lower.X);
            m[1] = 0.0f;
            m[2] = 0.0f;
            m[3] = 0.0f;

            m[4] = 0.0f;
            m[5] = 2.0f / (upper.Y - lower.Y);
            m[6] = 0.0f;
            m[7] = 0.0f;

            m[8] = 0.0f;
            m[9] = 0.0f;
            m[10] = 1.0f;
            m[11] = 0.0f;

            m[12] = -(upper.X + lower.X) / (upper.X - lower.X);
            m[13] = -(upper.Y + lower.Y) / (upper.Y - lower.Y);
            m[14] = zBias;
            m[15] = 1.0f;
        }
    }
}