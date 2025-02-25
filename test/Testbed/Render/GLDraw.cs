using System;
using Box2DSharp;
using ImGuiNET;
using OpenTK.Mathematics;
using Testbed.Abstractions;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.Render
{
    public class GLDraw : IDraw
    {
        public DebugDrawBase DebugDraw => _debugDraw;

        public bool ShowUI = true;

        private GLPoints _points;

        private GLLines _lines;

        private GLRenderTriangles _triangles;

        private GLCircles _circles = new();

        private GLSolidCapsules _solidCapsules;

        private GLSolidCircles _solidCircles;

        private GLSolidPolygons _solidPolygons = new();

        private GLBackground _background = new();

        private DebugDraw _debugDraw;

        public GLDraw()
        {
            _points = new();
            _lines = new();
            _triangles = new();
            _solidCapsules = new();
            _solidCircles = new();
            _solidPolygons = new();
            _debugDraw = new(this);
        }

        public void Dispose()
        {
            _points.Dispose();
            _points = null!;
            _lines.Dispose();
            _lines = null!;
            _triangles.Dispose();
            _triangles = null!;
            _solidCapsules.Dispose();
            _solidCapsules = null!;
            _solidCircles.Dispose();
            _solidCircles = null!;
            _solidPolygons.Dispose();
            _solidPolygons = null!;
            _circles.Dispose();
            _circles = null!;
            _background.Dispose();
            _background = null!;
            _debugDraw.Dispose();
            _debugDraw = null!;
        }

        public void Flush()
        {
            _solidCircles.Flush();
            _solidCapsules.Flush();
            _solidPolygons.Flush();
            _triangles.Flush();
            _circles.Flush();
            _lines.Flush();
            _points.Flush();
            RenderHelper.CheckGLError();
        }

        public void DrawPolygon(ReadOnlySpan<Vec2> vertices, int vertexCount, B2HexColor color)
        {
            if (vertexCount < 1)
            {
                return;
            }

            var p1 = vertices[vertexCount - 1];
            for (var i = 0; i < vertexCount; ++i)
            {
                var p2 = vertices[i];
                _lines.AddLine(p1, p2, color.ToColor4());
                p1 = p2;
            }
        }

        public void DrawSolidPolygon(Transform transform, ReadOnlySpan<Vec2> vertices, int vertexCount, float radius, B2HexColor color)
        {
            _solidPolygons.AddPolygon(transform, vertices, vertexCount, radius, color.ToColor4());
        }

        public void DrawCircle(Vec2 center, float radius, B2HexColor color)
        {
            _circles.AddCircle(center, radius, color.ToColor4());
        }

        public void DrawSolidCircle(Transform transform, Vec2 center, float radius, B2HexColor color)
        {
            _solidCircles.AddCircle(transform, radius, color.ToColor4());
        }

        public void DrawCapsule(Vec2 p1, Vec2 p2, float radius, B2HexColor color)
        {
            var (axis, length) = B2Math.Sub(p2, p1).GetLengthAndNormalize();

            if (length == 0.0f)
            {
                DrawCircle(p1, radius, color);
            }

            const float Segments = 16.0f;
            const float Increment = B2Math.Pi / Segments;
            float sinInc = MathF.Sin(Increment);
            float cosInc = MathF.Cos(Increment);

            Vec2 r1 = (-axis.Y, axis.X);
            Vec2 v1 = B2Math.MulAdd(p1, radius, r1);
            Vec2 a = v1;
            for (int i = 0; i < Segments; ++i)
            {
                // Perform rotation to avoid additional trigonometry.
                Vec2 r2;
                r2.X = cosInc * r1.X - sinInc * r1.Y;
                r2.Y = sinInc * r1.X + cosInc * r1.Y;
                Vec2 v2 = B2Math.MulAdd(p1, radius, r2);
                _lines.AddLine(v1, v2, color.ToColor4());
                r1 = r2;
                v1 = v2;
            }

            Vec2 b = v1;

            r1 = (axis.Y, -axis.X);
            v1 = B2Math.MulAdd(p2, radius, r1);
            Vec2 c = v1;
            for (int i = 0; i < Segments; ++i)
            {
                // Perform rotation to avoid additional trigonometry.
                Vec2 r2;
                r2.X = cosInc * r1.X - sinInc * r1.Y;
                r2.Y = sinInc * r1.X + cosInc * r1.Y;
                Vec2 v2 = B2Math.MulAdd(p2, radius, r2);
                _lines.AddLine(v1, v2, color.ToColor4());
                r1 = r2;
                v1 = v2;
            }

            Vec2 d = v1;
            _lines.AddLine(a, d, color.ToColor4());
            _lines.AddLine(b, c, color.ToColor4());
            _lines.AddLine(p1, p2, color.ToColor4());
        }

        public void DrawSolidCapsule(Vec2 p1, Vec2 p2, float radius, B2HexColor color)
        {
            _solidCapsules.AddCapsule(p1, p2, radius, color.ToColor4());
        }

        public void DrawSegment(Vec2 p1, Vec2 p2, B2HexColor color)
        {
            var color4 = color.ToColor4();
            _lines.AddLine(p1, p2, color4);
        }

        public void DrawTransform(Transform transform)
        {
            const float AxisScale = 0.4f;

            var p1 = transform.P.ToVector2();
            var p2 = p1 + AxisScale * transform.Q.GetXAxis().ToVector2();
            _lines.AddLine(p1, p2, Color4.Red);

            p2 = p1 + AxisScale * transform.Q.GetYAxis().ToVector2();
            _lines.AddLine(p1, p2, Color4.Green);
        }

        public void DrawPoint(Vec2 p, float size, B2HexColor color)
        {
            _points.AddPoint(p, size, color.ToColor4());
        }

        public void DrawString(int x, int y, string str)
        {
            ImGui.Begin("Overlay", ImGuiWindowFlags.NoTitleBar | ImGuiWindowFlags.NoInputs | ImGuiWindowFlags.AlwaysAutoResize | ImGuiWindowFlags.NoScrollbar);
            ImGui.SetCursorPos(new(x, y));
            ImGui.TextColored(new(0.9f, 0.6f, 0.6f, 1f), str);
            ImGui.End();
        }

        public void DrawString(Vec2 p, string str)
        {
            var ps = Global.Camera.ConvertWorldToScreen(p).ToVector2();
            ImGui.Begin("Overlay", ImGuiWindowFlags.NoTitleBar | ImGuiWindowFlags.NoInputs | ImGuiWindowFlags.AlwaysAutoResize | ImGuiWindowFlags.NoScrollbar);
            ImGui.SetCursorPos(ps);
            ImGui.TextColored(new(0.9f, 0.9f, 0.9f, 1), str);
            ImGui.End();
        }

        public void DrawAABB(AABB aabb, B2HexColor color)
        {
            var color4 = color.ToColor4();
            var p1 = aabb.LowerBound.ToVector2();
            var p2 = new Vector2(aabb.UpperBound.X, aabb.LowerBound.Y);
            var p3 = aabb.UpperBound.ToVector2();
            var p4 = new Vector2(aabb.LowerBound.X, aabb.UpperBound.Y);

            _lines.AddLine(p1, p2, color4);
            _lines.AddLine(p2, p3, color4);
            _lines.AddLine(p3, p4, color4);
            _lines.AddLine(p4, p1, color4);
        }

        public void DrawBackground()
        {
            _background.Draw();
        }
    }
}