using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Contacts;
using Testbed.Basics;

namespace Testbed.Tests
{
    [TestCase("Collision", "Sensors")]
    public class Sensors : Test
    {
        private const int Count = 7;

        private readonly Body[] _bodies = new Body[Count];

        private Fixture _sensor;

        public Sensors()
        {
            {
                var bd = new BodyDef();
                var ground = World.CreateBody(bd);

                {
                    var shape = new EdgeShape();
                    shape.Set(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
                    ground.CreateFixture(shape, 0.0f);
                }

                {
                    var shape = new CircleShape();
                    shape.Radius = 5.0f;
                    shape.Position.Set(0.0f, 10.0f);

                    var fd = new FixtureDef();
                    fd.Shape = shape;
                    fd.IsSensor = true;
                    _sensor = ground.CreateFixture(fd);
                }
            }

            {
                var shape = new CircleShape();
                shape.Radius = 1.0f;

                for (var i = 0; i < Count; ++i)
                {
                    var bd = new BodyDef();
                    bd.BodyType = BodyType.DynamicBody;
                    bd.Position.Set(-10.0f + 3.0f * i, 20.0f);
                    bd.UserData = false;
                    _bodies[i] = World.CreateBody(bd);
                    _bodies[i].CreateFixture(shape, 1.0f);
                }
            }
        }

        /// <inheritdoc />
        protected override void PreStep()
        {
            // Traverse the contact results. Apply a force on shapes
            // that overlap the sensor.
            for (var i = 0; i < Count; ++i)
            {
                var touching = (bool)_bodies[i].UserData;
                if (touching == false)
                {
                    continue;
                }

                var body = _bodies[i];
                var ground = _sensor.Body;

                var circle = (CircleShape)_sensor.Shape;
                var center = ground.GetWorldPoint(circle.Position);

                var position = body.GetPosition();

                var d = center - position;
                if (d.LengthSquared() < Settings.Epsilon * Settings.Epsilon)
                {
                    continue;
                }

                d = Vector2.Normalize(d);
                var F = 100.0f * d;
                body.ApplyForce(F, position, false);
            }
        }

        // Implement contact listener.
        public override void BeginContact(Contact contact)
        {
            var fixtureA = contact.FixtureA;
            var fixtureB = contact.FixtureB;

            if (fixtureA == _sensor)
            {
                var userData = fixtureB.Body.UserData;
                if (userData != null)
                {
                    userData = true;
                    fixtureB.Body.UserData = userData;
                }
            }

            if (fixtureB == _sensor)
            {
                var userData = fixtureA.Body.UserData;
                if (userData != null)
                {
                    userData = true;
                    fixtureA.Body.UserData = userData;
                }
            }
        }

        // Implement contact listener.
        public override void EndContact(Contact contact)
        {
            var fixtureA = contact.FixtureA;
            var fixtureB = contact.FixtureB;

            if (fixtureA == _sensor)
            {
                var userData = fixtureB.Body.UserData;
                if (userData != null)
                {
                    userData = false;
                    fixtureB.Body.UserData = userData;
                }
            }

            if (fixtureB == _sensor)
            {
                var userData = fixtureA.Body.UserData;
                if (userData != null)
                {
                    userData = false;
                    fixtureA.Body.UserData = userData;
                }
            }
        }
    }
}