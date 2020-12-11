using System.Threading;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Testbed.Abstractions;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.TestCases
{
    [TestCase("Solver", "Confined")]
    public class Confined : TestBase
    {
        private readonly int e_columnCount = 0;

        private readonly int e_rowCount = 0;

        private bool _autoCreate = false;

        public Confined()
        {
            {
                var bd = new BodyDef();
                var ground = World.CreateBody(bd);

                var shape = new EdgeShape();

                // Floor
                shape.SetTwoSided(new Vector2(-10.0f, 0.0f), new Vector2(10.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);

                // Left wall
                shape.SetTwoSided(new Vector2(-10.0f, 0.0f), new Vector2(-10.0f, 20.0f));
                ground.CreateFixture(shape, 0.0f);

                // Right wall
                shape.SetTwoSided(new Vector2(10.0f, 0.0f), new Vector2(10.0f, 20.0f));
                ground.CreateFixture(shape, 0.0f);

                // Roof
                shape.SetTwoSided(new Vector2(-10.0f, 20.0f), new Vector2(10.0f, 20.0f));
                ground.CreateFixture(shape, 0.0f);
            }
            {
                var radius = 0.5f;
                var shape = new CircleShape();
                shape.Position.SetZero();
                shape.Radius = radius;

                var fd = new FixtureDef();
                fd.Shape = shape;
                fd.Density = 1.0f;
                fd.Friction = 0.1f;

                for (var j = 0; j < e_columnCount; ++j)
                {
                    for (var i = 0; i < e_rowCount; ++i)
                    {
                        var bd = new BodyDef();
                        bd.BodyType = BodyType.DynamicBody;
                        bd.Position.Set(-10.0f + (2.1f * j + 1.0f + 0.01f * i) * radius, (2.0f * i + 1.0f) * radius);
                        var body = World.CreateBody(bd);

                        body.CreateFixture(fd);
                    }
                }
            }
            World.Gravity = new Vector2(0.0f, 0.0f);
        }

        private void CreateCircle()
        {
            var radius = 2.0f;
            var shape = new CircleShape();
            shape.Position.SetZero();
            shape.Radius = radius;

            var fd = new FixtureDef();
            fd.Shape = shape;
            fd.Density = 1.0f;
            fd.Friction = 0.0f;

            var p = new Vector2(RandomFloat(), 3.0f + RandomFloat());
            var bd = new BodyDef();
            bd.BodyType = BodyType.DynamicBody;
            bd.Position = p;

            var body = World.CreateBody(bd);

            body.CreateFixture(fd);
        }

        /// <inheritdoc />
        protected override void PreStep()
        {
            if (!_autoCreate)
            {
                return;
            }

            var sleeping = true;
            foreach (var b in World.BodyList)
            {
                if (b.BodyType != BodyType.DynamicBody)
                {
                    continue;
                }

                if (b.IsAwake)
                {
                    sleeping = false;
                }
            }

            if (sleeping)
            {
                Thread.Sleep(500);
                CreateCircle();
            }

            // foreach (var b in World.BodyList)
            // {
            //     if (b.BodyType != BodyType.DynamicBody)
            //     {
            //         continue;
            //     }
            //
            //     var p = b.GetPosition();
            //     if (p.X <= -10.0f || 10.0f <= p.X || p.Y <= 0.0f || 20.0f <= p.Y)
            //     {
            //         p.X += 0.1f;
            //     }
            // }
        }

        /// <inheritdoc />
        public override void OnKeyDown(KeyInputEventArgs keyInput)
        {
            if (keyInput.Key == KeyCodes.C)
            {
                CreateCircle();
            }

            if (keyInput.Key == KeyCodes.A)
            {
                _autoCreate = !_autoCreate;
            }
        }

        protected override void OnRender()
        {
            DrawString("Press 'c' to create a circle.");
            DrawString("Press 'a' to toggle auto generate circles.");
        }
    }
}