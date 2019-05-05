using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Inspection;
using UnityEngine;
using Vector2 = System.Numerics.Vector2;
using Color = System.Drawing.Color;

namespace Box2DSharp.Tests
{
    public class ConvexHull : Test
    {
        private const int Count = Settings.MaxPolygonVertices;

        private Vector2[] _points = new Vector2[Settings.MaxPolygonVertices];

        private int _count;

        private bool _auto;

        public ConvexHull()
        {
            Generate();
            _auto = false;
        }

        protected override void OnStep()
        {
            if (Input.GetKeyDown(KeyCode.A))
            {
                _auto = !_auto;
            }

            if (Input.GetKeyDown(KeyCode.G))
            {
                Generate();
            }
        }

        private readonly GUIStyle _style = new GUIStyle
        {
            fontSize = Screen.height * 2 / 100,
            alignment = TextAnchor.LowerLeft,
            normal =
            {
                textColor = new UnityEngine.Color(0.9f, 0.6f, 0.6f)
            }
        };

        public override void OnGUI()
        {
            foreach (var (position, text) in _contents)
            {
                var rect = new Rect(position.x, position.y, Screen.width, Screen.height * 2f / 100f);
                GUI.Label(rect, text, _style);
            }
        }

        public override void OnRender()
        {
            DrawString("Press g to generate a new random convex hull");
            DrawString("Press a to toggle random convex hull auto generation");
            var shape = new PolygonShape();
            shape.Set(_points);
            var drawLine = new Vector2[shape.Count + 1];
            Array.Copy(shape.Vertices.ToArray(), drawLine, shape.Count);
            drawLine[drawLine.Length - 1] = shape.Vertices[0];
            Drawer.DrawPolygon(drawLine, drawLine.Length, Color.FromArgb(230, 230, 230));
            _contents.Clear();
            var points = _points.Select(e => TestSettings.Camera.WorldToScreenPoint(e.ToUnityVector2()))
                                .Select(e => new UnityEngine.Vector2(e.x, Screen.height - e.y))
                                .ToArray();

            for (var i = 0; i < _count; ++i)
            {
                Drawer.DrawPoint(_points[i], 10.0f, Color.FromArgb(77, 230, 77));
                WriteString(points[i] + new UnityEngine.Vector2(0.5f, 0.5f), $"{i}");
            }

            Drawer.DrawPoint(Vector2.Zero, 5f, Color.Yellow);

            if (_auto && !TestSettings.Pause)
            {
                Generate();
            }
        }

        private readonly List<(UnityEngine.Vector2 position, string text)> _contents =
            new List<(UnityEngine.Vector2 position, string text)>();

        public void WriteString(UnityEngine.Vector2 position, string text)
        {
            _contents.Add((position, text));
        }

        void Generate()
        {
            var lowerBound = new Vector2(-8.0f, -8.0f);
            var upperBound = new Vector2(8.0f, 8.0f);

            for (var i = 0; i < Count; ++i)
            {
                var x = 10.0f * RandomFloat();
                var y = 10.0f * RandomFloat();

                // Clamp onto a square to help create collinearities.
                // This will stress the convex hull algorithm.
                var v = new Vector2(x, y);
                v = Vector2.Clamp(v, lowerBound, upperBound);
                _points[i] = v;
            }

            _count = Count;
        }
    }
}