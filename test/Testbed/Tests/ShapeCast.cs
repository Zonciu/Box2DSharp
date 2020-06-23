using System.Numerics;
using Box2DSharp.Collision;
using Box2DSharp.Common;
using Testbed.Basics;

namespace Testbed.Tests
{
    [TestCase("Collision", "Shape Cast")]
    public class ShapeCast : Test
    {
        private const int VertexCount = 8;

        private readonly Vector2[] _vAs = new Vector2[Settings.MaxPolygonVertices];

        private readonly Vector2[] _vBs = new Vector2[Settings.MaxPolygonVertices];

        private int _countA;

        private int _countB;

        private float _radiusA;

        private float _radiusB;

        public ShapeCast()
        {
            _vAs[0].Set(-0.5f, 1.0f);
            _vAs[1].Set(0.5f, 1.0f);
            _vAs[2].Set(0.0f, 0.0f);
            _countA = 3;
            _radiusA = Settings.PolygonRadius;

            _vBs[0].Set(-0.5f, -0.5f);
            _vBs[1].Set(0.5f, -0.5f);
            _vBs[2].Set(0.5f, 0.5f);
            _vBs[3].Set(-0.5f, 0.5f);
            _countB = 4;
            _radiusB = Settings.PolygonRadius;
        }

        protected override void OnRender()
        {
            var transformA = new Transform {Position = new Vector2(0.0f, 0.25f)};
            transformA.Rotation.SetIdentity();

            var transformB = new Transform();
            transformB.SetIdentity();

            var input = new ShapeCastInput();
            input.ProxyA.Set(_vAs, _countA, _radiusA);
            input.ProxyB.Set(_vBs, _countB, _radiusB);
            input.TransformA = transformA;
            input.TransformB = transformB;
            input.TranslationB.Set(8.0f, 0.0f);
            var hit = DistanceAlgorithm.ShapeCast(out var output, input);

            var transformB2 = new Transform
            {
                Rotation = transformB.Rotation, Position = transformB.Position + output.Lambda * input.TranslationB
            };

            var distanceInput = new DistanceInput
            {
                TransformA = transformA,
                TransformB = transformB2,
                UseRadii = false
            };
            distanceInput.ProxyA.Set(_vAs, _countA, _radiusA);
            distanceInput.ProxyB.Set(_vBs, _countB, _radiusB);
            var simplexCache = new SimplexCache();

            DistanceAlgorithm.Distance(out var distanceOutput, ref simplexCache, distanceInput);
            DrawString(
                $"hit = {hit}, iters = {output.Iterations}, lambda = {output.Lambda}, distance = {distanceOutput.Distance}");

            var vertices = new Vector2[Settings.MaxPolygonVertices];

            for (var i = 0; i < _countA; ++i)
            {
                vertices[i] = MathUtils.Mul(transformA, _vAs[i]);
            }

            //g_debugDraw.DrawCircle(vertices[0], _radiusA, b2Color(0.9f, 0.9f, 0.9f));
            Drawer.DrawPolygon(vertices, _countA, Color.FromArgb(230, 230, 230));

            for (var i = 0; i < _countB; ++i)
            {
                vertices[i] = MathUtils.Mul(transformB, _vBs[i]);
            }

            //g_debugDraw.DrawCircle(vertices[0], _radiusB, b2Color(0.5f, 0.9f, 0.5f));
            Drawer.DrawPolygon(vertices, _countB, Color.FromArgb(127, 230, 127));

            for (var i = 0; i < _countB; ++i)
            {
                vertices[i] = MathUtils.Mul(transformB2, _vBs[i]);
            }

            //g_debugDraw.DrawCircle(vertices[0], _radiusB, b2Color(0.5f, 0.7f, 0.9f));
            Drawer.DrawPolygon(vertices, _countB, Color.FromArgb(127, 129, 230));

            if (hit)
            {
                var p1 = output.Point;
                Drawer.DrawPoint(p1, 10.0f, Color.FromArgb(230, 77, 77));
                var p2 = p1 + output.Normal;
                Drawer.DrawSegment(p1, p2, Color.FromArgb(230, 77, 77));
            }
        }
    }
}