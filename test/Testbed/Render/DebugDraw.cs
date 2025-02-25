using System;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Render;

public class DebugDraw(IDraw draw) : DebugDrawBase, IDisposable
{
    private IDraw _draw = draw;

    public void Flush()
    {
        _draw.Flush();
    }

    public void Dispose()
    {
        _draw = null!;
    }

    public override void DrawPolygon(ReadOnlySpan<Vec2> vertices, int vertexCount, B2HexColor color, object? context)
    {
        _draw.DrawPolygon(vertices, vertexCount, color);
    }

    public override void DrawSolidPolygon(Transform transform, ReadOnlySpan<Vec2> vertices, int vertexCount, float radius, B2HexColor color, object? context)
    {
        _draw.DrawSolidPolygon(transform, vertices, vertexCount, radius, color);
    }

    public override void DrawCircle(Vec2 center, float radius, B2HexColor color, object? context)
    {
        _draw.DrawCircle(center, radius, color);
    }

    public override void DrawSolidCircle(Transform transform, float radius, B2HexColor color, object? context)
    {
        _draw.DrawSolidCircle(transform, Vec2.Zero, radius, color);
    }

    public override void DrawCapsule(Vec2 p1, Vec2 p2, float radius, B2HexColor color, object? context)
    {
        _draw.DrawCapsule(p1, p2, radius, color);
    }

    public override void DrawSolidCapsule(Vec2 p1, Vec2 p2, float radius, B2HexColor color, object? context)
    {
        _draw.DrawSolidCapsule(p1, p2, radius, color);
    }

    public override void DrawSegment(Vec2 p1, Vec2 p2, B2HexColor color, object? context)
    {
        _draw.DrawSegment(p1, p2, color);
    }

    public override void DrawTransform(Transform transform, object? context)
    {
        _draw.DrawTransform(transform);
    }

    public override void DrawPoint(Vec2 p, float size, B2HexColor color, object? context)
    {
        _draw.DrawPoint(p, size, color);
    }

    public override void DrawString(Vec2 p, string s, object? context)
    {
        _draw.DrawString(p, s);
    }

    public override void DrawBackground()
    {
        _draw.DrawBackground();
    }
}