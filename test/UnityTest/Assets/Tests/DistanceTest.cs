using Box2DSharp.Collision;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using UnityEngine;
using Color = System.Drawing.Color;
using Transform = Box2DSharp.Common.Transform;
using Vector2 = System.Numerics.Vector2;

namespace Box2DSharp.Tests
{
    public class DistanceTest : TestBase
    {
        private float _angleB;

        private PolygonShape _polygonA = new PolygonShape();

        private PolygonShape _polygonB = new PolygonShape();

        private Vector2 _positionB;

        private Transform _transformA;

        private Transform _transformB;

        protected override void Create()
        {
            {
                _transformA.SetIdentity();
                _transformA.Position.Set(0.0f, -0.2f);
                _polygonA.SetAsBox(10.0f, 0.2f);
            }

            {
                _positionB.Set(12.017401f, 0.13678508f);
                _angleB = -0.0109265f;
                _transformB.Set(_positionB, _angleB);

                _polygonB.SetAsBox(2.0f, 0.1f);
            }
        }

        /// <inheritdoc />
        protected override void PostStep()
        {
            if (Input.GetKeyDown(KeyCode.A))
            {
                _positionB.X -= 0.1f;
            }

            if (Input.GetKeyDown(KeyCode.D))
            {
                _positionB.X += 0.1f;
            }

            if (Input.GetKeyDown(KeyCode.S))
            {
                _positionB.Y -= 0.1f;
            }

            if (Input.GetKeyDown(KeyCode.W))
            {
                _positionB.Y += 0.1f;
            }

            if (Input.GetKeyDown(KeyCode.Q))
            {
                _angleB += 0.1f * Settings.Pi;
            }

            if (Input.GetKeyDown(KeyCode.E))
            {
                _angleB -= 0.1f * Settings.Pi;
            }

            _transformB.Set(_positionB, _angleB);

            var input = new DistanceInput();
            input.ProxyA.Set(_polygonA, 0);
            input.ProxyB.Set(_polygonB, 0);
            input.TransformA = _transformA;
            input.TransformB = _transformB;
            input.UseRadii = true;
            var cache = SimplexCache.Create();
            cache.Count = 0;
            DistanceAlgorithm.Distance(out var output, ref cache, input);

            DrawString($"distance = {output.Distance}");

            DrawString($"iterations = {output.Iterations}");

            {
                var color = Color.FromArgb(230, 230, 230);
                var v = new Vector2[Settings.MaxPolygonVertices];
                for (var i = 0; i < _polygonA.Count; ++i)
                {
                    v[i] = MathUtils.Mul(_transformA, _polygonA.Vertices[i]);
                }

                Drawer.DrawPolygon(v, _polygonA.Count, color);

                for (var i = 0; i < _polygonB.Count; ++i)
                {
                    v[i] = MathUtils.Mul(_transformB, _polygonB.Vertices[i]);
                }

                Drawer.DrawPolygon(v, _polygonB.Count, color);
            }

            var x1 = output.PointA;
            var x2 = output.PointB;

            var c1 = Color.FromArgb(255, 0, 0);
            Drawer.DrawPoint(x1, 4.0f, c1);

            var c2 = Color.FromArgb(255, 255, 0);
            Drawer.DrawPoint(x2, 4.0f, c2);
        }
    }
}