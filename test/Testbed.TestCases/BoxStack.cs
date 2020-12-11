using Box2DSharp.Collision.Shapes;
using Box2DSharp.Dynamics;
using Testbed.Abstractions;
using Debug = System.Diagnostics.Debug;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.TestCases
{
    [TestCase("Stacking", "Boxes")]
    public class BoxStack : TestBase
    {
        private const int ColumnCount = 1;

        private const int RowCount = 15;

        private Body _bullet;

        private readonly Body[] _bodies = new Body [RowCount * ColumnCount];

        private readonly int[] _indices = new int[RowCount * ColumnCount];

        public BoxStack()
        {
            {
                var bd = new BodyDef();
                var ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);

                shape.SetTwoSided(new Vector2(20.0f, 0.0f), new Vector2(20.0f, 20.0f));
                ground.CreateFixture(shape, 0.0f);
            }

            var xs = new float[5]
            {
                0.0f, -10.0f, -5.0f, 5.0f, 10.0f
            };
            for (var j = 0; j < ColumnCount; ++j)
            {
                var shape = new PolygonShape();
                shape.SetAsBox(0.5f, 0.5f);

                var fd = new FixtureDef
                {
                    Shape = shape, Density = 1.0f,
                    Friction = 0.3f
                };

                for (var i = 0; i < RowCount; ++i)
                {
                    var bd = new BodyDef {BodyType = BodyType.DynamicBody};

                    var n = j * RowCount + i;
                    Debug.Assert(n < RowCount * ColumnCount);
                    _indices[n] = n;
                    bd.UserData = _indices[n];

                    var x = 0.0f;

                    //var x = RandomFloat(-0.02f, 0.02f);
                    //var x = i % 2 == 0 ? -0.01f : 0.01f;
                    bd.Position = new Vector2(xs[j] + x, 0.55f + 1.1f * i);
                    var body = World.CreateBody(bd);

                    _bodies[n] = body;

                    body.CreateFixture(fd);
                }
            }

            _bullet = null;
        }

        /// <inheritdoc />
        /// <inheritdoc />
        public override void OnKeyDown(KeyInputEventArgs keyInput)
        {
            if (keyInput.Key == KeyCodes.F)
            {
                if (_bullet != null)
                {
                    World.DestroyBody(_bullet);
                    _bullet = null;
                }

                {
                    var shape = new CircleShape {Radius = 0.25f};

                    var fd = new FixtureDef
                    {
                        Shape = shape, Density = 20.0f,
                        Restitution = 0.05f
                    };

                    var bd = new BodyDef
                    {
                        BodyType = BodyType.DynamicBody, Bullet = true,
                        Position = new Vector2(-31.0f, 5.0f)
                    };

                    _bullet = World.CreateBody(bd);
                    _bullet.CreateFixture(fd);

                    _bullet.SetLinearVelocity(new Vector2(400.0f, 0.0f));
                }
            }
        }

        protected override void OnRender()
        {
            DrawString("Press F to launch a bullet");
        }
    }
}