using System.Numerics;
using System.Threading.Tasks;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Benchmark", "ManyTumblers")]
    public class ManyTumblers : TestBase
    {
        private class Context
        {
            public int BodyIndex;

            public int StepCount;

            public Body[] Bodies;
        }

        private int _tumblerCount;

        private Vector2[] _tumblerPositions;

        private Body[] _tumblerBodies;

        private int _rowCount = 5;

        private int _columnCount = 5;

        private World[] _worlds;

        private Context[] _contexts;

        public override void OnInitialize()
        {
            Global.Camera.Center = new Vector2(1.0f, -5.5f);
            Global.Camera.Zoom = 1;
            TestSettings.DrawJoints = false;
            CreateScene();
        }

        public override void Dispose()
        {
            foreach (var world in _worlds)
            {
                world.Dispose();
            }

            base.Dispose();
        }

        protected override void PreStep()
        {
            Parallel.For(
                0,
                _worlds.Length,
                (int index) =>
                {
                    var ctx = _contexts[index];
                    var world = _worlds[index];
                    world.Step(TimeStep, TestSettings.VelocityIterations, TestSettings.PositionIterations);
                    if (ctx.BodyIndex < ctx.Bodies.Length && (StepCount & 0x7) == 0)
                    {
                        var box = new PolygonShape();
                        box.SetAsBox(0.125f, 0.125f, Vector2.Zero, 0);

                        var bodyDef = new BodyDef();
                        bodyDef.BodyType = BodyType.DynamicBody;
                        bodyDef.Position = _tumblerPositions[index];
                        var body = world.CreateBody(bodyDef);
                        body.CreateFixture(box, 1);
                        ctx.Bodies[ctx.BodyIndex] = body;
                        ctx.BodyIndex += 1;
                    }
                });
        }

        protected override void OnRender()
        {
            foreach (var world in _worlds)
            {
                world.DebugDraw();
            }
        }

        private void CreateTumbler(World world, Vector2 position)
        {
            var bodyDef = new BodyDef();
            bodyDef.BodyType = BodyType.KinematicBody;
            bodyDef.Position = position;
            bodyDef.AngularVelocity = Settings.Pi / 180.0f * 15;
            var body = world.CreateBody(bodyDef);

            var shapeDef = new FixtureDef();
            shapeDef.Density = 50.0f;

            var polygon = new PolygonShape();
            polygon.SetAsBox(0.25f, 2f, new Vector2(2, 0), 0);
            shapeDef.Shape = polygon;

            body.CreateFixture(shapeDef);

            polygon.SetAsBox(0.25f, 2f, new Vector2(-2, 0), 0);
            body.CreateFixture(shapeDef);

            polygon.SetAsBox(2.0f, 0.25f, new Vector2(0.0f, 2.0f), 0);
            body.CreateFixture(shapeDef);

            polygon.SetAsBox(2.0f, 0.25f, new Vector2(0.0f, -2.0f), 0);
            body.CreateFixture(shapeDef);
        }

        private void CreateScene()
        {
            _tumblerCount = _rowCount * _columnCount;
            _worlds = new World[_tumblerCount];
            _contexts = new Context[_tumblerCount];
            _tumblerBodies = new Body[_tumblerCount];
            _tumblerPositions = new Vector2[_tumblerCount];
            var bodiesPerTumbler = 30;

            var index = 0;
            var x = -4.0f * _rowCount;
            for (var i = 0; i < _rowCount; ++i)
            {
                var y = -4.0f * _columnCount;
                for (var j = 0; j < _columnCount; ++j)
                {
                    _tumblerPositions[index] = new Vector2(x, y);
                    var world = new World();
                    world.Draw = Draw;
                    _worlds[index] = world;
                    _contexts[index] = new Context
                    {
                        Bodies = new Body[bodiesPerTumbler]
                    };
                    CreateTumbler(world, _tumblerPositions[index]);
                    ++index;
                    y += 8.0f;
                }

                x += 8.0f;
            }
        }
    }
}