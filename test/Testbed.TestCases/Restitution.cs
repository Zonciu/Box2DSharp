using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Forces", "Restitution")]
    public class Restitution : TestBase
    {
        public Restitution()
        {
            const float threshold = 10.0f;
            {
                var bd = new BodyDef();
                var ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));

                FixtureDef fd = new FixtureDef();
                fd.Shape = shape;
                fd.RestitutionThreshold = threshold;
                ground.CreateFixture(fd);
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
                    fd.RestitutionThreshold = threshold;
                    body.CreateFixture(fd);
                }
            }
        }
    }
}