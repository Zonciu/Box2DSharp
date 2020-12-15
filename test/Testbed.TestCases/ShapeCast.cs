using System.Numerics;
using Box2DSharp.Collision;
using Box2DSharp.Common;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Collision", "Shape Cast")]
    public class ShapeCast : TestBase
    {
        private const int VertexCount = 8;

        private readonly Vector2[] _vAs = new Vector2[Settings.MaxPolygonVertices];

        private readonly Vector2[] _vBs = new Vector2[Settings.MaxPolygonVertices];

        private int _countA;

        private int _countB;

        private float _radiusA;

        private float _radiusB;

        private Transform _transformA;

        private Transform _transformB;

        private Vector2 _translationB;

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

            _transformA.Position.Set(0.0f, 0.25f);
            _transformA.Rotation.SetIdentity();
            _transformB.Position.Set(-4.0f, 0.0f);
            _transformB.Rotation.SetIdentity();
            _translationB.Set(8.0f, 0.0f);
        }

        protected override void OnRender()
        {
            var input = new ShapeCastInput();
            input.ProxyA.Set(_vAs, _countA, _radiusA);
            input.ProxyB.Set(_vBs, _countB, _radiusB);
            input.TransformA = _transformA;
            input.TransformB = _transformB;
            input.TranslationB = _translationB;
            var hit = DistanceAlgorithm.ShapeCast(out var output, input);

            var transformB2 = new Transform
            {
                Rotation = _transformB.Rotation,
                Position = _transformB.Position + output.Lambda * input.TranslationB
            };

            var distanceInput = new DistanceInput
            {
                TransformA = _transformA,
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
                vertices[i] = MathUtils.Mul(_transformA, _vAs[i]);
            }

            if (_countA == 1)
            {
                Drawer.DrawCircle(vertices[0], _radiusA, Color.FromArgb(0.9f, 0.9f, 0.9f));
            }
            else
            {
                Drawer.DrawPolygon(vertices, _countA, Color.FromArgb(0.9f, 0.9f, 0.9f));
            }

            for (var i = 0; i < _countB; ++i)
            {
                vertices[i] = MathUtils.Mul(_transformB, _vBs[i]);
            }

            if (_countB == 1)
            {
                Drawer.DrawCircle(vertices[0], _radiusB, Color.FromArgb(0.5f, 0.9f, 0.5f));
            }
            else
            {
                Drawer.DrawPolygon(vertices, _countB, Color.FromArgb(0.5f, 0.9f, 0.5f));
            }

            for (var i = 0; i < _countB; ++i)
            {
                vertices[i] = MathUtils.Mul(transformB2, _vBs[i]);
            }

            if (_countB == 1)
            {
                Drawer.DrawCircle(vertices[0], _radiusB, Color.FromArgb(0.5f, 0.7f, 0.9f));
            }
            else
            {
                Drawer.DrawPolygon(vertices, _countB, Color.FromArgb(0.5f, 0.7f, 0.9f));
            }

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