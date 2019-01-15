using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;

namespace Box2DSharp.Tests
{
    public class CompoundShapes : TestBase
    {
        protected override void Create()
        {
            {
                var bd = new BodyDef();
                bd.Position.Set(0.0f, 0.0f);
                var body = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.Set(new Vector2(50.0f, 0.0f), new Vector2(-50.0f, 0.0f));

                body.CreateFixture(shape, 0.0f);
            }

            {
                var circle1 = new CircleShape();
                circle1.Radius = 0.5f;
                circle1.Position.Set(-0.5f, 0.5f);

                var circle2 = new CircleShape();
                circle2.Radius = 0.5f;
                circle2.Position.Set(0.5f, 0.5f);

                for (var i = 0; i < 10; ++i)
                {
                    var x = RandomFloat(-0.1f, 0.1f);
                    var bd = new BodyDef();
                    bd.BodyType = BodyType.DynamicBody;
                    bd.Position.Set(x + 5.0f, 1.05f + 2.5f * i);
                    bd.Angle = RandomFloat(-Settings.Pi, Settings.Pi);
                    var body = World.CreateBody(bd);
                    body.CreateFixture(circle1, 2.0f);
                    body.CreateFixture(circle2, 0.0f);
                }
            }

            {
                var polygon1 = new PolygonShape();
                polygon1.SetAsBox(0.25f, 0.5f);

                var polygon2 = new PolygonShape();
                polygon2.SetAsBox(0.25f, 0.5f, new Vector2(0.0f, -0.5f), 0.5f * Settings.Pi);

                for (var i = 0; i < 10; ++i)
                {
                    var x = RandomFloat(-0.1f, 0.1f);
                    var bd = new BodyDef();
                    bd.BodyType = BodyType.DynamicBody;
                    bd.Position.Set(x - 5.0f, 1.05f + 2.5f * i);
                    bd.Angle = RandomFloat(-Settings.Pi, Settings.Pi);
                    var body = World.CreateBody(bd);
                    body.CreateFixture(polygon1, 2.0f);
                    body.CreateFixture(polygon2, 2.0f);
                }
            }

            {
                var xf1 = new Transform();
                xf1.Rotation.Set(0.3524f * Settings.Pi);
                xf1.Position = xf1.Rotation.GetXAxis();

                var vertices = new Vector2[3];

                var triangle1 = new PolygonShape();
                vertices[0] = MathUtils.Mul(xf1, new Vector2(-1.0f, 0.0f));
                vertices[1] = MathUtils.Mul(xf1, new Vector2(1.0f, 0.0f));
                vertices[2] = MathUtils.Mul(xf1, new Vector2(0.0f, 0.5f));
                triangle1.Set(vertices, 3);

                var xf2 = new Transform();
                xf2.Rotation.Set(-0.3524f * Settings.Pi);
                xf2.Position = -xf2.Rotation.GetXAxis();

                var triangle2 = new PolygonShape();
                vertices[0] = MathUtils.Mul(xf2, new Vector2(-1.0f, 0.0f));
                vertices[1] = MathUtils.Mul(xf2, new Vector2(1.0f, 0.0f));
                vertices[2] = MathUtils.Mul(xf2, new Vector2(0.0f, 0.5f));
                triangle2.Set(vertices, 3);

                for (var i = 0; i < 10; ++i)
                {
                    var x = RandomFloat(-0.1f, 0.1f);
                    var bd = new BodyDef();
                    bd.BodyType = BodyType.DynamicBody;
                    bd.Position.Set(x, 2.05f + 2.5f * i);
                    bd.Angle = 0.0f;
                    var body = World.CreateBody(bd);
                    body.CreateFixture(triangle1, 2.0f);
                    body.CreateFixture(triangle2, 2.0f);
                }
            }

            {
                var bottom = new PolygonShape();
                bottom.SetAsBox(1.5f, 0.15f);

                var left = new PolygonShape();
                left.SetAsBox(0.15f, 2.7f, new Vector2(-1.45f, 2.35f), 0.2f);

                var right = new PolygonShape();
                right.SetAsBox(0.15f, 2.7f, new Vector2(1.45f, 2.35f), -0.2f);

                var bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(0.0f, 2.0f);
                var body = World.CreateBody(bd);
                body.CreateFixture(bottom, 4.0f);
                body.CreateFixture(left, 4.0f);
                body.CreateFixture(right, 4.0f);
            }
        }
    }
}