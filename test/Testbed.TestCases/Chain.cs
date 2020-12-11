using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Joints", "Chain")]
    public class Chain : TestBase
    {
        public Chain()
        {
            Body ground;
            {
                var bd = new BodyDef();
                ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);
            }

            {
                var shape = new PolygonShape();
                shape.SetAsBox(0.6f, 0.125f);

                var fd = new FixtureDef
                {
                    Shape = shape,
                    Density = 20.0f,
                    Friction = 0.2f
                };

                var jd = new RevoluteJointDef {CollideConnected = false};

                const float y = 25.0f;
                var prevBody = ground;
                for (var i = 0; i < 30; ++i)
                {
                    var bd = new BodyDef {BodyType = BodyType.DynamicBody};
                    bd.Position.Set(0.5f + i, y);
                    var body = World.CreateBody(bd);
                    body.CreateFixture(fd);

                    var anchor = new Vector2(i, y);
                    jd.Initialize(prevBody, body, anchor);
                    World.CreateJoint(jd);

                    prevBody = body;
                }
            }
        }
    }
}