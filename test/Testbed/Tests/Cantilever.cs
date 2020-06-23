using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Testbed.Basics;

namespace Testbed.Tests
{
    [TestCase("Joints", "Cantilever")]
    public class Cantilever : Test
    {
        private const int Count = 8;

        public Cantilever()
        {
            Body ground;
            {
                var bd = new BodyDef();
                ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.Set(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);
            }

            {
                var shape = new PolygonShape();
                shape.SetAsBox(0.5f, 0.125f);

                var fd = new FixtureDef();
                fd.Shape = shape;
                fd.Density = 20.0f;

                var jd = new WeldJointDef();

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

                    prevBody = body;
                }
            }

            {
                var shape = new PolygonShape();
                shape.SetAsBox(1.0f, 0.125f);

                var fd = new FixtureDef();
                fd.Shape = shape;
                fd.Density = 20.0f;

                var jd = new WeldJointDef();
                jd.FrequencyHz = 5.0f;
                jd.DampingRatio = 0.7f;

                var prevBody = ground;
                for (var i = 0; i < 3; ++i)
                {
                    var bd = new BodyDef();
                    bd.BodyType = BodyType.DynamicBody;
                    bd.Position.Set(-14.0f + 2.0f * i, 15.0f);
                    var body = World.CreateBody(bd);
                    body.CreateFixture(fd);

                    var anchor = new Vector2(-15.0f + 2.0f * i, 15.0f);
                    jd.Initialize(prevBody, body, anchor);
                    World.CreateJoint(jd);

                    prevBody = body;
                }
            }

            {
                var shape = new PolygonShape();
                shape.SetAsBox(0.5f, 0.125f);

                var fd = new FixtureDef();
                fd.Shape = shape;
                fd.Density = 20.0f;

                var jd = new WeldJointDef();

                var prevBody = ground;
                for (var i = 0; i < Count; ++i)
                {
                    var bd = new BodyDef();
                    bd.BodyType = BodyType.DynamicBody;
                    bd.Position.Set(-4.5f + 1.0f * i, 5.0f);
                    var body = World.CreateBody(bd);
                    body.CreateFixture(fd);

                    if (i > 0)
                    {
                        var anchor = new Vector2(-5.0f + 1.0f * i, 5.0f);
                        jd.Initialize(prevBody, body, anchor);
                        World.CreateJoint(jd);
                    }

                    prevBody = body;
                }
            }

            {
                var shape = new PolygonShape();
                shape.SetAsBox(0.5f, 0.125f);

                var fd = new FixtureDef();
                fd.Shape = shape;
                fd.Density = 20.0f;

                var jd = new WeldJointDef();
                jd.FrequencyHz = 8.0f;
                jd.DampingRatio = 0.7f;

                var prevBody = ground;
                for (var i = 0; i < Count; ++i)
                {
                    var bd = new BodyDef();
                    bd.BodyType = BodyType.DynamicBody;
                    bd.Position.Set(5.5f + 1.0f * i, 10.0f);
                    var body = World.CreateBody(bd);
                    body.CreateFixture(fd);

                    if (i > 0)
                    {
                        var anchor = new Vector2(5.0f + 1.0f * i, 10.0f);
                        jd.Initialize(prevBody, body, anchor);
                        World.CreateJoint(jd);
                    }

                    prevBody = body;
                }
            }

            for (var i = 0; i < 2; ++i)
            {
                var vertices = new Vector2 [3];
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

            for (var i = 0; i < 2; ++i)
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