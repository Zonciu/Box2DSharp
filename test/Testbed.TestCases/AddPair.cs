using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Benchmark", "Add Pair Stress Test")]
    public class AddPair : TestBase
    {
        public AddPair()
        {
            World.Gravity = new Vector2(0.0f, 0.0f);
            {
                var shape = new CircleShape();
                shape.Position.SetZero();
                shape.Radius = 0.1f;

                var minX = -6.0f;
                var maxX = 0.0f;
                var minY = 4.0f;
                var maxY = 6.0f;

                for (var i = 0; i < 400; ++i)
                {
                    var bd = new BodyDef();
                    bd.BodyType = BodyType.DynamicBody;
                    bd.Position = new Vector2(RandomFloat(minX, maxX), RandomFloat(minY, maxY));
                    var body = World.CreateBody(bd);
                    body.CreateFixture(shape, 0.01f);
                }
            }

            {
                var shape = new PolygonShape();
                shape.SetAsBox(1.5f, 1.5f);
                var bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(-40.0f, 5.0f);
                bd.Bullet = true;
                var body = World.CreateBody(bd);
                body.CreateFixture(shape, 1.0f);
                body.SetLinearVelocity(new Vector2(10.0f, 0.0f));
            }
        }
    }
}