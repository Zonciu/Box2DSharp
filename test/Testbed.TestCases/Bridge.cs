using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Joints", "Bridge")]
    public class Bridge : TestBase
    {
        public const int Count = 30;

        private Body _middle;

        public Bridge()
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
                shape.SetAsBox(0.5f, 0.125f);

                var fd = new FixtureDef();
                fd.Shape = shape;
                fd.Density = 20.0f;
                fd.Friction = 0.2f;

                var jd = new RevoluteJointDef();

                var prevBody = ground;
                for (var i = 0; i < Count; ++i)
                {
                    var bd = new BodyDef();
                    bd.BodyType = BodyType.DynamicBody;
                    bd.Position.Set(-14.5f + 1.0f * i, 5.0f);
                    var body = World.CreateBody(bd);
                    body.CreateFixture(fd);

                    var anchor = new Vector2(-15.0f + 1.0f * i, 5.0f);
                    jd.Initialize(prevBody, body, anchor);
                    World.CreateJoint(jd);

                    if (i == Count >> 1)
                    {
                        _middle = body;
                    }

                    prevBody = body;
                }

                {
                    var anchor = new Vector2(-15.0f + 1.0f * Count, 5.0f);
                    jd.Initialize(prevBody, ground, anchor);
                    World.CreateJoint(jd);
                }
            }

            for (var i = 0; i < 2; ++i)
            {
                var vertices = new Vector2[3];
                vertices[0].Set(-0.5f, 0.0f);
                vertices[1].Set(0.5f, 0.0f);
                vertices[2].Set(0.0f, 1.5f);

                var shape = new PolygonShape();
                shape.Set(vertices);

                var fd = new FixtureDef();
                fd.Shape = shape;
                fd.Density = 1.0f;

                var bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(-8.0f + 8.0f * i, 12.0f);
                var body = World.CreateBody(bd);
                body.CreateFixture(fd);
            }

            for (var i = 0; i < 3; ++i)
            {
                var shape = new CircleShape();
                shape.Radius = 0.5f;

                var fd = new FixtureDef();
                fd.Shape = shape;
                fd.Density = 1.0f;

                var bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(-6.0f + 6.0f * i, 10.0f);
                var body = World.CreateBody(bd);
                body.CreateFixture(fd);
            }
        }
    }
}