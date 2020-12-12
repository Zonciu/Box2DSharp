using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using Box2DSharp.Collision;
using Box2DSharp.Common;
using Box2DSharp.Testbed.Unity.Inspection;
using Testbed.Abstractions;
using UnityEngine;
using Color = Box2DSharp.Common.Color;
using Transform = Box2DSharp.Common.Transform;
using Vector2 = System.Numerics.Vector2;
using Vector3 = UnityEngine.Vector3;

namespace Box2DSharp.Testbed.Unity
{
    public class DebugDrawer : IDebugDrawer
    {
        public UnityDrawer Drawer;

        /// <inheritdoc />
        public DrawFlag Flags { get; set; }

        public bool ShowUI = true;

        /// <inheritdoc />
        public void DrawPolygon(Span<Vector2> vertices, int vertexCount, in Color color)
        {
            var list = new List<(Vector3 begin, Vector3 end)>();

            for (var i = 0; i < vertexCount; i++)
            {
                if (i < vertexCount - 1)
                {
                    list.Add((vertices[i].ToUnityVector3(), vertices[i + 1].ToUnityVector3()));
                }
                else
                {
                    list.Add((vertices[i].ToUnityVector3(), vertices[0].ToUnityVector3()));
                }
            }

            Drawer.PostLines(list, color.ToUnityColor());
        }

        /// <inheritdoc />
        public void DrawSolidPolygon(Span<Vector2> vertices, int vertexCount, in Color color)
        {
            var list = new List<(Vector3 begin, Vector3 end)>();

            for (var i = 0; i < vertexCount; i++)
            {
                if (i < vertexCount - 1)
                {
                    list.Add((vertices[i].ToUnityVector3(), vertices[i + 1].ToUnityVector3()));
                }
                else
                {
                    list.Add((vertices[i].ToUnityVector3(), vertices[0].ToUnityVector3()));
                }
            }

            Drawer.PostLines(list, color.ToUnityColor());
        }

        /// <inheritdoc />
        public void DrawCircle(in Vector2 center, float radius, in Color color)
        {
            var lines = new List<(Vector3, Vector3)>();
            const int lineCount = 100;
            for (var i = 0; i <= lineCount; ++i) //割圆术画圆
            {
                lines.Add(
                    (
                        new UnityEngine.Vector2(
                            center.X + radius * (float)Math.Cos(2 * Mathf.PI / lineCount * i),
                            center.Y + radius * (float)Math.Sin(2 * Mathf.PI / lineCount * i)),
                        new UnityEngine.Vector2(
                            center.X + radius * (float)Math.Cos(2 * Mathf.PI / lineCount * (i + 1)),
                            center.Y + radius * (float)Math.Sin(2 * Mathf.PI / lineCount * (i + 1)))
                    ));
            }

            Drawer.PostLines(lines, color.ToUnityColor());
        }

        /// <inheritdoc />
        public void DrawSolidCircle(in Vector2 center, float radius, in Vector2 axis, in Color color)
        {
            var lines = new List<(Vector3, Vector3)>();
            const int lineCount = 100;
            for (var i = 0; i <= lineCount; ++i) //割圆术画圆
            {
                lines.Add(
                    (
                        new UnityEngine.Vector2(
                            center.X + radius * (float)Math.Cos(2 * Mathf.PI / lineCount * i),
                            center.Y + radius * (float)Math.Sin(2 * Mathf.PI / lineCount * i)),
                        new UnityEngine.Vector2(
                            center.X + radius * (float)Math.Cos(2 * Mathf.PI / lineCount * (i + 1)),
                            center.Y + radius * (float)Math.Sin(2 * Mathf.PI / lineCount * (i + 1)))
                    ));
            }

            Drawer.PostLines(lines, color.ToUnityColor());
            var p = center + radius * axis;
            DrawSegment(center, p, color);
        }

        /// <inheritdoc />
        public void DrawSegment(in Vector2 p1, in Vector2 p2, in Color color)
        {
            Drawer.PostLines(
                new List<(Vector3, Vector3)> {(p1.ToUnityVector2(), p2.ToUnityVector2())},
                color.ToUnityColor());
        }

        /// <inheritdoc />
        public void DrawTransform(in Transform xf)
        {
            const float axisScale = 0.4f;

            var p1 = xf.Position;
            var p2 = p1 + axisScale * xf.Rotation.GetXAxis();

            Drawer.PostLines(
                new List<(Vector3, Vector3)> {(p1.ToUnityVector2(), p2.ToUnityVector2())},
                UnityEngine.Color.red);

            p2 = p1 + axisScale * xf.Rotation.GetYAxis();
            Drawer.PostLines(
                new List<(Vector3 begin, Vector3 end)> {(p1.ToUnityVector2(), p2.ToUnityVector2())},
                UnityEngine.Color.green);
        }

        /// <inheritdoc />
        public void DrawPoint(in Vector2 p, float size, in Color color)
        {
            Drawer.PostPoint((p.ToUnityVector3(), size / 100, color.ToUnityColor()));
        }

        /// <inheritdoc />
        public void DrawAABB(AABB aabb, Color color)
        {
            var p1 = aabb.LowerBound;
            var p2 = new Vector2(aabb.UpperBound.X, aabb.LowerBound.Y);
            var p3 = aabb.UpperBound;
            var p4 = new Vector2(aabb.LowerBound.X, aabb.UpperBound.Y);
            DrawPolygon(new[] {p1, p2, p3, p4}, 4, color);
        }

        public readonly ConcurrentQueue<(Vector2 Position, string Text)> Texts = new ConcurrentQueue<(Vector2 Position, string Text)>();

        /// <inheritdoc />
        public void DrawString(float x, float y, string strings)
        {
            Texts.Enqueue((new Vector2(x, y), strings));
        }

        /// <inheritdoc />
        public void DrawString(int x, int y, string strings)
        {
            Texts.Enqueue((new Vector2(x, y), strings));
        }

        /// <inheritdoc />
        public void DrawString(Vector2 position, string strings)
        {
            Texts.Enqueue((position, strings));
        }
    }
}