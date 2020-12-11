using Box2DSharp.Collision.Shapes;
using Box2DSharp.Dynamics;
using Testbed.Abstractions;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.TestCases
{
    [TestCase("Solver", "Heavy 2")]
    public class Heavy2 : TestBase
    {
        private Body _heavy;

        public Heavy2()
        {
            {
                var bd = new BodyDef();
                var ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);
            }
            {
                var bd = new BodyDef
                {
                    BodyType = BodyType.DynamicBody,
                    Position = new Vector2(0.0f, 2.5f)
                };
                var body = World.CreateBody(bd);

                var shape = new CircleShape {Radius = 0.5f};
                body.CreateFixture(shape, 10.0f);

                bd.Position = new Vector2(0.0f, 3.5f);
                body = World.CreateBody(bd);
                body.CreateFixture(shape, 10.0f);
            }
            _heavy = null;
        }

        void ToggleHeavy()
        {
            if (_heavy != null)
            {
                World.DestroyBody(_heavy);
                _heavy = null;
            }
            else
            {
                var bd = new BodyDef
                {
                    BodyType = BodyType.DynamicBody,
                    Position = new Vector2(0.0f, 9.0f)
                };
                _heavy = World.CreateBody(bd);

                var shape = new CircleShape {Radius = 5.0f};
                _heavy.CreateFixture(shape, 10.0f);
            }
        }

        /// <inheritdoc />
        public override void OnKeyDown(KeyInputEventArgs keyInput)
        {
            if (keyInput.Key == KeyCodes.H)
            {
                ToggleHeavy();
            }
        }

        protected override void OnRender()
        {
            DrawString("Press H to place heavy body");
        }
    }
}