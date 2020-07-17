using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Testbed.Basics;

namespace Testbed.Tests
{
    [TestCase("Forces", "Restitution")]
    public class Restitution : Test
    {
        public Restitution()
        {
            {
                var bd = new BodyDef();
                var ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);
            }

            {
                var shape = new CircleShape();
                shape.Radius = 1.0f;

                var fd = new FixtureDef();
                fd.Shape = shape;
                fd.Density = 1.0f;

                float[] restitution = {0.0f, 0.1f, 0.3f, 0.5f, 0.75f, 0.9f, 1.0f};

                for (var i = 0; i < 7; ++i)
                {
                    var bd = new BodyDef();
                    bd.BodyType = BodyType.DynamicBody;
                    bd.Position.Set(-10.0f + 3.0f * i, 20.0f);

                    var body = World.CreateBody(bd);

                    fd.Restitution = restitution[i];
                    body.CreateFixture(fd);
                }
            }
        }
    }
}