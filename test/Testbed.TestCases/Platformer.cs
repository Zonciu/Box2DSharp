using System.Numerics;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Contacts;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Examples", "Platformer")]
    public class Platformer : TestBase
    {
        private Fixture _character;

        private Fixture _platform;

        private float _radius;

        private float _top;

        public Platformer()
        {
            // Ground
            {
                var bd = new BodyDef();
                var ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(-20.0f, 0.0f), new Vector2(20.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);
            }

            // Platform
            {
                var bd = new BodyDef();
                bd.Position.Set(0.0f, 10.0f);
                var body = World.CreateBody(bd);

                var shape = new PolygonShape();
                shape.SetAsBox(3.0f, 0.5f);
                _platform = body.CreateFixture(shape, 0.0f);

                _top = 10.0f + 0.5f;
            }

            // Actor
            {
                var bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(0.0f, 12.0f);
                var body = World.CreateBody(bd);

                _radius = 0.5f;
                var shape = new CircleShape();
                shape.Radius = _radius;
                _character = body.CreateFixture(shape, 20.0f);

                body.SetLinearVelocity(new Vector2(0.0f, -50.0f));
            }
        }

        /// <inheritdoc />
        public override void PreSolve(Contact contact, in Manifold oldManifold)
        {
            base.PreSolve(contact, oldManifold);
            var fixtureA = contact.FixtureA;
            var fixtureB = contact.FixtureB;

            if (fixtureA != _platform && fixtureA != _character)
            {
                return;
            }

            if (fixtureB != _platform && fixtureB != _character)
            {
                return;
            }

            var position = _character.Body.GetPosition();

            if (position.Y < _top + _radius - 3.0f * Settings.LinearSlop)
            {
                contact.SetEnabled(false);
            }
        }

        protected override void OnRender()
        {
            var v = _character.Body.LinearVelocity;
            DrawString($"Character Linear Velocity: {v.Y}");
        }
    }
}