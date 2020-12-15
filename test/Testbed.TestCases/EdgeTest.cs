using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Geometry", "Edge Test")]
    public class EdgeTest : TestBase
    {
        public readonly Vector2 Offset1;

        public readonly Vector2 Offset2;

        public Body Body1;

        public Body Body2;

        public bool Boxes;

        public EdgeTest()
        {
            var vertices = new Vector2[]
            {
                new Vector2(10.0f, -4.0f),
                new Vector2(10.0f, 0.0f),
                new Vector2(6.0f, 0.0f),
                new Vector2(4.0f, 2.0f),
                new Vector2(2.0f, 0.0f),
                new Vector2(-2.0f, 0.0f),
                new Vector2(-6.0f, 0.0f),
                new Vector2(-8.0f, -3.0f),
                new Vector2(-10.0f, 0.0f),
                new Vector2(-10.0f, -4.0f)
            };

            Offset1.Set(0.0f, 8.0f);
            Offset2.Set(0.0f, 16.0f);

            {
                var v1 = vertices[0] + Offset1;
                var v2 = vertices[1] + Offset1;
                var v3 = vertices[2] + Offset1;
                var v4 = vertices[3] + Offset1;
                var v5 = vertices[4] + Offset1;
                var v6 = vertices[5] + Offset1;
                var v7 = vertices[6] + Offset1;
                var v8 = vertices[7] + Offset1;
                var v9 = vertices[8] + Offset1;
                var v10 = vertices[9] + Offset1;

                BodyDef bd = new BodyDef();
                Body ground = World.CreateBody(bd);

                var shape = new EdgeShape();

                shape.SetOneSided(v10, v1, v2, v3);
                ground.CreateFixture(shape, 0.0f);

                shape.SetOneSided(v1, v2, v3, v4);
                ground.CreateFixture(shape, 0.0f);

                shape.SetOneSided(v2, v3, v4, v5);
                ground.CreateFixture(shape, 0.0f);

                shape.SetOneSided(v3, v4, v5, v6);
                ground.CreateFixture(shape, 0.0f);

                shape.SetOneSided(v4, v5, v6, v7);
                ground.CreateFixture(shape, 0.0f);

                shape.SetOneSided(v5, v6, v7, v8);
                ground.CreateFixture(shape, 0.0f);

                shape.SetOneSided(v6, v7, v8, v9);
                ground.CreateFixture(shape, 0.0f);

                shape.SetOneSided(v7, v8, v9, v10);
                ground.CreateFixture(shape, 0.0f);

                shape.SetOneSided(v8, v9, v10, v1);
                ground.CreateFixture(shape, 0.0f);

                shape.SetOneSided(v9, v10, v1, v2);
                ground.CreateFixture(shape, 0.0f);
            }

            {
                var v1 = vertices[0] + Offset2;
                var v2 = vertices[1] + Offset2;
                var v3 = vertices[2] + Offset2;
                var v4 = vertices[3] + Offset2;
                var v5 = vertices[4] + Offset2;
                var v6 = vertices[5] + Offset2;
                var v7 = vertices[6] + Offset2;
                var v8 = vertices[7] + Offset2;
                var v9 = vertices[8] + Offset2;
                var v10 = vertices[9] + Offset2;

                BodyDef bd = new BodyDef();
                Body ground = World.CreateBody(bd);

                EdgeShape shape = new EdgeShape();

                shape.SetTwoSided(v1, v2);
                ground.CreateFixture(shape, 0.0f);

                shape.SetTwoSided(v2, v3);
                ground.CreateFixture(shape, 0.0f);

                shape.SetTwoSided(v3, v4);
                ground.CreateFixture(shape, 0.0f);

                shape.SetTwoSided(v4, v5);
                ground.CreateFixture(shape, 0.0f);

                shape.SetTwoSided(v5, v6);
                ground.CreateFixture(shape, 0.0f);

                shape.SetTwoSided(v6, v7);
                ground.CreateFixture(shape, 0.0f);

                shape.SetTwoSided(v7, v8);
                ground.CreateFixture(shape, 0.0f);

                shape.SetTwoSided(v8, v9);
                ground.CreateFixture(shape, 0.0f);

                shape.SetTwoSided(v9, v10);
                ground.CreateFixture(shape, 0.0f);

                shape.SetTwoSided(v10, v1);
                ground.CreateFixture(shape, 0.0f);
            }

            Body1 = null;
            Body2 = null;
            CreateBoxes();
            Boxes = true;
        }

        protected void CreateBoxes()
        {
            if (Body1 != null)
            {
                World.DestroyBody(Body1);
                Body1 = null;
            }

            if (Body2 != null)
            {
                World.DestroyBody(Body2);
                Body2 = null;
            }

            {
                BodyDef bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position = new Vector2(8.0f, 2.6f) + Offset1;
                bd.AllowSleep = false;
                Body1 = World.CreateBody(bd);

                PolygonShape shape = new PolygonShape();
                shape.SetAsBox(0.5f, 1.0f);

                Body1.CreateFixture(shape, 1.0f);
            }

            {
                BodyDef bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position = new Vector2(8.0f, 2.6f) + Offset2;
                bd.AllowSleep = false;
                Body2 = World.CreateBody(bd);

                PolygonShape shape = new PolygonShape();
                shape.SetAsBox(0.5f, 1.0f);

                Body2.CreateFixture(shape, 1.0f);
            }
        }

        protected void CreateCircles()
        {
            if (Body1 != null)
            {
                World.DestroyBody(Body1);
                Body1 = null;
            }

            if (Body2 != null)
            {
                World.DestroyBody(Body2);
                Body2 = null;
            }

            {
                BodyDef bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position = new Vector2(-0.5f, 0.6f) + Offset1;
                bd.AllowSleep = false;
                Body1 = World.CreateBody(bd);

                CircleShape shape = new CircleShape();
                shape.Radius = 0.5f;

                Body1.CreateFixture(shape, 1.0f);
            }

            {
                BodyDef bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position = new Vector2(-0.5f, 0.6f) + Offset2;
                bd.AllowSleep = false;
                Body2 = World.CreateBody(bd);

                CircleShape shape = new CircleShape();
                shape.Radius = 0.5f;

                Body2.CreateFixture(shape, 1.0f);
            }
        }

        /// <inheritdoc />
        public override void OnKeyDown(KeyInputEventArgs keyInput)
        {
            if (keyInput.Key == KeyCodes.A)
            {
                Body1.ApplyForceToCenter(new Vector2(-10.0f, 0.0f), true);
                Body2.ApplyForceToCenter(new Vector2(-10.0f, 0.0f), true);
            }

            if (keyInput.Key == KeyCodes.D)
            {
                Body1.ApplyForceToCenter(new Vector2(10.0f, 0.0f), true);
                Body2.ApplyForceToCenter(new Vector2(10.0f, 0.0f), true);
            }
        }
    }
}