using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Contacts;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Collision", "Sensors")]
    public class Sensors : TestBase
    {
        private const int Count = 7;

        private readonly Body[] _bodies = new Body[Count];

        protected float _force;

        private bool[] _touching = new bool[Count];

        private Fixture _sensor;

        public Sensors()
        {
            {
                var bd = new BodyDef();
                var ground = World.CreateBody(bd);

                {
                    var shape = new EdgeShape();
                    shape.SetTwoSided(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
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
                    bd.UserData = i;
                    _touching[i] = false;
                    _bodies[i] = World.CreateBody(bd);
                    _bodies[i].CreateFixture(shape, 1.0f);
                }
            }
            _force = 100.0f;
        }

        /// <inheritdoc />
        protected override void PreStep()
        {
            // Traverse the contact results. Apply a force on shapes
            // that overlap the sensor.
            for (var i = 0; i < Count; ++i)
            {
                if (_touching[i] == false)
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
                var F = _force * d;
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
                var index = (int?)fixtureB.Body.UserData;
                if (index < Count)
                {
                    _touching[index.Value] = true;
                }
            }

            if (fixtureB == _sensor)
            {
                var index = (int?)fixtureA.Body.UserData;
                if (index < Count)
                {
                    _touching[index.Value] = true;
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
                var index = (int?)fixtureB.Body.UserData;
                if (index < Count)
                {
                    _touching[index.Value] = false;
                }
            }

            if (fixtureB == _sensor)
            {
                var index = (int?)fixtureA.Body.UserData;
                if (index < Count)
                {
                    _touching[index.Value] = false;
                }
            }
        }
    }
}