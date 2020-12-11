using System;
using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Examples", "Character Collision")]
    public class CharacterCollision : TestBase
    {
        private Body _character;

        public CharacterCollision()
        {
            // Ground body
            {
                var bd = new BodyDef();
                var ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(-20.0f, 0.0f), new Vector2(20.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);
            }

            // Collinear edges with no adjacency information.
            // This shows the problematic case where a box shape can hit
            // an internal vertex.
            {
                var bd = new BodyDef();
                var ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(-8.0f, 1.0f), new Vector2(-6.0f, 1.0f));
                ground.CreateFixture(shape, 0.0f);
                shape.SetTwoSided(new Vector2(-6.0f, 1.0f), new Vector2(-4.0f, 1.0f));
                ground.CreateFixture(shape, 0.0f);
                shape.SetTwoSided(new Vector2(-4.0f, 1.0f), new Vector2(-2.0f, 1.0f));
                ground.CreateFixture(shape, 0.0f);
            }

            // Chain shape
            {
                var bd = new BodyDef {Angle = 0.25f * Settings.Pi};
                var ground = World.CreateBody(bd);

                var vs = new Vector2[4]
                {
                    new Vector2(5.0f, 7.0f),
                    new Vector2(6.0f, 8.0f),
                    new Vector2(7.0f, 8.0f),
                    new Vector2(8.0f, 7.0f)
                };
                var shape = new ChainShape();
                shape.CreateLoop(vs);
                ground.CreateFixture(shape, 0.0f);
            }

            // Square tiles. This shows that adjacency shapes may
            // have non-smooth collision. There is no solution
            // to this problem.
            {
                var bd = new BodyDef();
                var ground = World.CreateBody(bd);

                var shape = new PolygonShape();
                shape.SetAsBox(1.0f, 1.0f, new Vector2(4.0f, 3.0f), 0.0f);
                ground.CreateFixture(shape, 0.0f);
                shape.SetAsBox(1.0f, 1.0f, new Vector2(6.0f, 3.0f), 0.0f);
                ground.CreateFixture(shape, 0.0f);
                shape.SetAsBox(1.0f, 1.0f, new Vector2(8.0f, 3.0f), 0.0f);
                ground.CreateFixture(shape, 0.0f);
            }

            // Square made from an edge loop. Collision should be smooth.
            {
                var bd = new BodyDef();
                var ground = World.CreateBody(bd);

                var vs = new Vector2[4]
                {
                    new Vector2(-1.0f, 3.0f),
                    new Vector2(1.0f, 3.0f),
                    new Vector2(1.0f, 5.0f),
                    new Vector2(-1.0f, 5.0f)
                };

                var shape = new ChainShape();
                shape.CreateLoop(vs);
                ground.CreateFixture(shape, 0.0f);
            }

            // Edge loop. Collision should be smooth.
            {
                var bd = new BodyDef {Position = new Vector2(-10.0f, 4.0f)};
                var ground = World.CreateBody(bd);

                var vs = new Vector2[10]
                {
                    new Vector2(0.0f, 0.0f),
                    new Vector2(6.0f, 0.0f),
                    new Vector2(6.0f, 2.0f),
                    new Vector2(4.0f, 1.0f),
                    new Vector2(2.0f, 2.0f),
                    new Vector2(0.0f, 2.0f),
                    new Vector2(-2.0f, 2.0f),
                    new Vector2(-4.0f, 3.0f),
                    new Vector2(-6.0f, 2.0f),
                    new Vector2(-6.0f, 0.0f)
                };
                var shape = new ChainShape();
                shape.CreateLoop(vs);
                ground.CreateFixture(shape, 0.0f);
            }

            // Square character 1
            {
                var bd = new BodyDef
                {
                    Position = new Vector2(-3.0f, 8.0f),
                    BodyType = BodyType.DynamicBody,
                    FixedRotation = true,
                    AllowSleep = false
                };

                var body = World.CreateBody(bd);

                var shape = new PolygonShape();
                shape.SetAsBox(0.5f, 0.5f);

                var fd = new FixtureDef {Shape = shape, Density = 20.0f};
                body.CreateFixture(fd);
            }

            // Square character 2
            {
                var bd = new BodyDef
                {
                    Position = new Vector2(-5.0f, 5.0f),
                    BodyType = BodyType.DynamicBody,
                    FixedRotation = true,
                    AllowSleep = false
                };

                var body = World.CreateBody(bd);

                var shape = new PolygonShape();
                shape.SetAsBox(0.25f, 0.25f);

                var fd = new FixtureDef {Shape = shape, Density = 20.0f};
                body.CreateFixture(fd);
            }

            // Hexagon character
            {
                var bd = new BodyDef
                {
                    Position = new Vector2(-5.0f, 8.0f),
                    BodyType = BodyType.DynamicBody,
                    FixedRotation = true,
                    AllowSleep = false
                };

                var body = World.CreateBody(bd);

                var angle = 0.0f;
                const float delta = Settings.Pi / 3.0f;
                var vertices = new Vector2[6];
                for (var i = 0; i < 6; ++i)
                {
                    vertices[i].Set(0.5f * (float)Math.Cos(angle), 0.5f * (float)Math.Sin(angle));
                    angle += delta;
                }

                var shape = new PolygonShape();
                shape.Set(vertices);

                var fd = new FixtureDef {Shape = shape, Density = 20.0f};
                body.CreateFixture(fd);
            }

            // Circle character
            {
                var bd = new BodyDef
                {
                    Position = new Vector2(3.0f, 5.0f),
                    BodyType = BodyType.DynamicBody,
                    FixedRotation = true,
                    AllowSleep = false
                };

                var body = World.CreateBody(bd);

                var shape = new CircleShape {Radius = 0.5f};

                var fd = new FixtureDef {Shape = shape, Density = 20.0f};
                body.CreateFixture(fd);
            }

            // Circle character
            {
                var bd = new BodyDef
                {
                    Position = new Vector2(-7.0f, 6.0f),
                    BodyType = BodyType.DynamicBody,
                    AllowSleep = false
                };

                _character = World.CreateBody(bd);

                var shape = new CircleShape {Radius = 0.25f};

                var fd = new FixtureDef
                {
                    Shape = shape,
                    Density = 20.0f,
                    Friction = 1.0f
                };
                _character.CreateFixture(fd);
            }
        }

        /// <inheritdoc />
        protected override void PreStep()
        {
            var v = _character.LinearVelocity;
            v.X = -5.0f;
            _character.SetLinearVelocity(v);
        }

        protected override void OnRender()
        {
            DrawString("This tests various character collision shapes.");
            DrawString("Limitation: square and hexagon can snag on aligned boxes.");
            DrawString("Feature: edge chains have smooth collision inside and out.");
        }
    }
}