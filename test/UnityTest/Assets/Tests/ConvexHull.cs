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
    public class ConvexHull : TestBase
    {
        private const int e_count = Settings.MaxPolygonVertices;

        Vector2[] m_points = new Vector2[Settings.MaxPolygonVertices];

        private int m_count;

        private bool m_auto;

        private PolygonShape Shape;

        protected override void Create()
        {
            Generate();
            m_auto = false;

            // {
            //     var bd = new BodyDef()
            //     {
            //         BodyType = BodyType.StaticBody
            //     };
            //     var body = World.CreateBody(bd);
            //
            //     Shape = new PolygonShape();
            //     Shape.Set(m_points);
            //     Shape = (PolygonShape) body.CreateFixture(Shape, 0.0f).Shape;
            // }
        }

        /// <inheritdoc />
        protected override void PreUpdate()
        {
            if (Input.GetKeyDown(KeyCode.A))
            {
                m_auto = !m_auto;
            }

            if (Input.GetKeyDown(KeyCode.G))
            {
                Generate();
            }
        }

        /// <inheritdoc />
        protected override void PostStep()
        {
            //            Shape.Set(m_points);
            DrawString("Press g to generate a new random convex hull");

            // Todo
            var shape = new PolygonShape();
            shape.Set(m_points);
            var drawLine = new Vector2[shape.Count + 1];
            Array.Copy(shape.Vertices, drawLine, shape.Count);
            drawLine[drawLine.Length - 1] = shape.Vertices[0];
            Drawer.DrawPolygon(drawLine, drawLine.Length, Color.FromArgb(230, 230, 230));
            _contents.Clear();

            var points = m_points.Select(e => MainCamera.WorldToScreenPoint(e.ToUnityVector2()))
                                 .Select(e => new UnityEngine.Vector2(e.x, Screen.height - e.y))
                                 .ToArray();
            for (var i = 0; i < m_count; ++i)
            {
                Drawer.DrawPoint(m_points[i], 10.0f, Color.FromArgb(77, 230, 77));

                WriteString(points[i] + new UnityEngine.Vector2(0.5f, 0.5f), $"{i}");
            }

            Drawer.DrawPoint(Vector2.Zero, 5f, Color.Yellow);

            //
            // if (shape.Validate() == false)
            // {
            //     //m_textLine += 0;
            // }

            if (m_auto)
            {
                Generate();
            }
        }

        private ConvexHull()
        {
            Generate();
            m_auto = false;
        }

        private readonly List<(UnityEngine.Vector2 position, string text)> _contents =
            new List<(UnityEngine.Vector2 position, string text)>();

        public void WriteString(UnityEngine.Vector2 position, string text)
        {
            _contents.Add((position, text));
        }

        /// <inheritdoc />
        protected override void OnGUI()
        {
            base.OnGUI();
            foreach (var (position, text) in _contents)
            {
                var rect = new Rect(position.x, position.y, Screen.width, Screen.height);
                var style = new GUIStyle
                {
                    fontSize = Screen.height * 2 / 100,
                    normal = {textColor = new UnityEngine.Color(0.9f, 0.6f, 0.6f)}
                };
                GUI.Label(rect, text, style);
            }
        }

        void Generate()
        {
            var lowerBound = new Vector2(-8.0f, -8.0f);
            var upperBound = new Vector2(8.0f, 8.0f);

            for (var i = 0; i < e_count; ++i)
            {
                var x = 10.0f * RandomFloat();
                var y = 10.0f * RandomFloat();

                // Clamp onto a square to help create collinearities.
                // This will stress the convex hull algorithm.
                var v = new Vector2(x, y);
                v = Vector2.Clamp(v, lowerBound, upperBound);
                m_points[i] = v;
            }

            m_count = e_count;
        }
    }
}