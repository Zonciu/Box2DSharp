using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Testbed.Basics;

namespace Testbed.Tests
{
    [TestCase("Geometry", "Edge Test")]
    public class EdgeTest : Test
    {
        public EdgeTest()
        {
            {
                var bd = new BodyDef();
                var ground = World.CreateBody(bd);

                Vector2 v1 = new Vector2(-10.0f, 0.0f), v2 = new Vector2(-7.0f, -2.0f), v3 = new Vector2(-4.0f, 0.0f);
                Vector2 v4 = new Vector2(0.0f, 0.0f),
                        v5 = new Vector2(4.0f, 0.0f),
                        v6 = new Vector2(7.0f, 2.0f),
                        v7 = new Vector2(10.0f, 0.0f);

                var shape = new EdgeShape();

                shape.Set(v1, v2);
                shape.HasVertex3 = true;
                shape.Vertex3 = v3;
                ground.CreateFixture(shape, 0.0f);

                shape.Set(v2, v3);
                shape.HasVertex0 = true;
                shape.HasVertex3 = true;
                shape.Vertex0 = v1;
                shape.Vertex3 = v4;
                ground.CreateFixture(shape, 0.0f);

                shape.Set(v3, v4);
                shape.HasVertex0 = true;
                shape.HasVertex3 = true;
                shape.Vertex0 = v2;
                shape.Vertex3 = v5;
                ground.CreateFixture(shape, 0.0f);

                shape.Set(v4, v5);
                shape.HasVertex0 = true;
                shape.HasVertex3 = true;
                shape.Vertex0 = v3;
                shape.Vertex3 = v6;
                ground.CreateFixture(shape, 0.0f);

                shape.Set(v5, v6);
                shape.HasVertex0 = true;
                shape.HasVertex3 = true;
                shape.Vertex0 = v4;
                shape.Vertex3 = v7;
                ground.CreateFixture(shape, 0.0f);

                shape.Set(v6, v7);
                shape.HasVertex0 = true;
                shape.Vertex0 = v5;
                ground.CreateFixture(shape, 0.0f);
            }

            {
                var bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(-0.5f, 0.6f);
                bd.AllowSleep = false;
                var body = World.CreateBody(bd);

                var shape = new CircleShape();
                shape.Radius = 0.5f;

                body.CreateFixture(shape, 1.0f);
            }

            {
                var bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(1.0f, 0.6f);
                bd.AllowSleep = false;
                var body = World.CreateBody(bd);

                var shape = new PolygonShape();
                shape.SetAsBox(0.5f, 0.5f);

                body.CreateFixture(shape, 1.0f);
            }
        }
    }
}