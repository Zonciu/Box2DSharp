using System.Numerics;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Collision", "Time of Impact")]
    public class TimeOfImpactTest : TestBase
    {
        private PolygonShape _shapeA = new PolygonShape();

        private PolygonShape _shapeB = new PolygonShape();

        public int ToiMaxIters => World.ToiProfile.ToiMaxIters;

        public int ToiMaxRootIters => World.ToiProfile.ToiMaxRootIters;

        public TimeOfImpactTest()
        {
            World.ToiProfile = new ToiProfile
            {
                World = World
            };
            _shapeA.SetAsBox(25.0f, 5.0f);
            _shapeB.SetAsBox(2.5f, 2.5f);
        }

        protected override void OnRender()
        {
            var sweepA = new Sweep();
            sweepA.C0.Set(24.0f, -60.0f);
            sweepA.A0 = 2.95f;
            sweepA.C = sweepA.C0;
            sweepA.A = sweepA.A0;
            sweepA.LocalCenter.SetZero();

            var sweepB = new Sweep();
            sweepB.C0.Set(53.474274f, -50.252514f);
            sweepB.A0 = 513.36676f; // - 162.0f * _pi;
            sweepB.C.Set(54.595478f, -51.083473f);
            sweepB.A = 513.62781f; //  - 162.0f * _pi;
            sweepB.LocalCenter.SetZero();

            //sweepB.A0 -= 300.0f * _pi;
            //sweepB.A -= 300.0f * _pi;

            var input = new ToiInput();
            input.ProxyA.Set(_shapeA, 0);
            input.ProxyB.Set(_shapeB, 0);
            input.SweepA = sweepA;
            input.SweepB = sweepB;
            input.Tmax = 1.0f;

            TimeOfImpact.ComputeTimeOfImpact(out var output, input, World.ToiProfile, World.GJkProfile);

            DrawString($"toi = {output.Time}");

            DrawString($"max toi iters = {ToiMaxIters}, max root iters = {ToiMaxRootIters}");

            var vertices = new Vector2[Settings.MaxPolygonVertices];

            sweepA.GetTransform(out var transformA, 0.0f);

            for (var i = 0; i < _shapeA.Count; ++i)
            {
                vertices[i] = MathUtils.Mul(transformA, _shapeA.Vertices[i]);
            }

            Drawer.DrawPolygon(vertices, _shapeA.Count, Color.FromArgb(230, 230, 230));

            sweepB.GetTransform(out var transformB, 0.0f);

            //Vec2 localPoint(2.0f, -0.1f);

            for (var i = 0; i < _shapeB.Count; ++i)
            {
                vertices[i] = MathUtils.Mul(transformB, _shapeB.Vertices[i]);
            }

            Drawer.DrawPolygon(vertices, _shapeB.Count, Color.FromArgb(127, 230, 127));

            sweepB.GetTransform(out transformB, output.Time);

            for (var i = 0; i < _shapeB.Count; ++i)
            {
                vertices[i] = MathUtils.Mul(transformB, _shapeB.Vertices[i]);
            }

            Drawer.DrawPolygon(vertices, _shapeB.Count, Color.FromArgb(127, 178, 230));

            sweepB.GetTransform(out transformB, 1.0f);

            for (var i = 0; i < _shapeB.Count; ++i)
            {
                vertices[i] = MathUtils.Mul(transformB, _shapeB.Vertices[i]);
            }

            Drawer.DrawPolygon(vertices, _shapeB.Count, Color.FromArgb(230, 127, 127));
        }
    }
}