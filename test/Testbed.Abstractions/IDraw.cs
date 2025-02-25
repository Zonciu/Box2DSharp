using System;
using Box2DSharp;

namespace Testbed.Abstractions
{
    public interface IDraw : IDisposable
    {
        DebugDrawBase DebugDraw { get; }

        void DrawPolygon(ReadOnlySpan<Vec2> vertices, int vertexCount, B2HexColor color);

        void DrawSolidPolygon(Transform transform, ReadOnlySpan<Vec2> vertices, int vertexCount, float radius, B2HexColor color);

        void DrawCircle(Vec2 center, float radius, B2HexColor color);

        void DrawSolidCircle(Transform transform, Vec2 center, float radius, B2HexColor color);

        void DrawCapsule(Vec2 p1, Vec2 p2, float radius, B2HexColor color);

        void DrawSolidCapsule(Vec2 p1, Vec2 p2, float radius, B2HexColor color);

        void DrawSegment(Vec2 p1, Vec2 p2, B2HexColor color);

        void DrawTransform(Transform transform);

        void DrawPoint(Vec2 p, float size, B2HexColor color);

        void DrawString(int x, int y, string str);

        void DrawString(Vec2 p, string str);

        void DrawAABB(AABB aabb, B2HexColor color);

        void Flush();

        void DrawBackground();
    }
}