using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Dynamics;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Solver", "Heavy 1")]
    public class Heavy1 : TestBase
    {
        public Heavy1()
        {
            {
                var bd = new BodyDef();
                var ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);
            }
            {
                var bd = new BodyDef {BodyType = BodyType.DynamicBody, Position = new Vector2(0.0f, 0.5f)};
                var body = World.CreateBody(bd);

                var shape = new CircleShape {Radius = 0.5f};
                body.CreateFixture(shape, 10.0f);

                bd.Position = new Vector2(0.0f, 6.0f);
                body = World.CreateBody(bd);
                shape.Radius = 5.0f;
                body.CreateFixture(shape, 10.0f);
            }
        }
    }
}