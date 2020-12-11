using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Testbed.Abstractions;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.TestCases
{
    [TestCase("Examples", "Slider Crank 2")]
    public class SliderCrank2 : TestBase
    {
        private RevoluteJoint _joint1;

        private PrismaticJoint _joint2;

        public SliderCrank2()
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
                var prevBody = ground;

                // Define crank.
                {
                    var shape = new PolygonShape();
                    shape.SetAsBox(0.5f, 2.0f);

                    var bd = new BodyDef();
                    bd.BodyType = BodyType.DynamicBody;
                    bd.Position = new Vector2(0.0f, 7.0f);
                    var body = World.CreateBody(bd);
                    body.CreateFixture(shape, 2.0f);

                    var rjd = new RevoluteJointDef();
                    rjd.Initialize(prevBody, body, new Vector2(0.0f, 5.0f));
                    rjd.MotorSpeed = 1.0f * Settings.Pi;
                    rjd.MaxMotorTorque = 10000.0f;
                    rjd.EnableMotor = true;
                    _joint1 = (RevoluteJoint)World.CreateJoint(rjd);

                    prevBody = body;
                }

                // Define follower.
                {
                    var shape = new PolygonShape();
                    shape.SetAsBox(0.5f, 4.0f);

                    var bd = new BodyDef {BodyType = BodyType.DynamicBody, Position = new Vector2(0.0f, 13.0f)};
                    var body = World.CreateBody(bd);
                    body.CreateFixture(shape, 2.0f);

                    var rjd = new RevoluteJointDef();
                    rjd.Initialize(prevBody, body, new Vector2(0.0f, 9.0f));
                    rjd.EnableMotor = false;
                    World.CreateJoint(rjd);

                    prevBody = body;
                }

                // Define piston
                {
                    var shape = new PolygonShape();
                    shape.SetAsBox(1.5f, 1.5f);

                    var bd = new BodyDef
                    {
                        BodyType = BodyType.DynamicBody, FixedRotation = true,
                        Position = new Vector2(0.0f, 17.0f)
                    };
                    var body = World.CreateBody(bd);
                    body.CreateFixture(shape, 2.0f);

                    var rjd = new RevoluteJointDef();
                    rjd.Initialize(prevBody, body, new Vector2(0.0f, 17.0f));
                    World.CreateJoint(rjd);

                    var pjd = new PrismaticJointDef();
                    pjd.Initialize(ground, body, new Vector2(0.0f, 17.0f), new Vector2(0.0f, 1.0f));

                    pjd.MaxMotorForce = 1000.0f;
                    pjd.EnableMotor = true;

                    _joint2 = (PrismaticJoint)World.CreateJoint(pjd);
                }

                // Create a payload
                {
                    var shape = new PolygonShape();
                    shape.SetAsBox(1.5f, 1.5f);

                    var bd = new BodyDef();
                    bd.BodyType = BodyType.DynamicBody;
                    bd.Position = new Vector2(0.0f, 23.0f);
                    var body = World.CreateBody(bd);
                    body.CreateFixture(shape, 2.0f);
                }
            }
        }

        /// <inheritdoc />
        /// <inheritdoc />
        public override void OnKeyDown(KeyInputEventArgs keyInput)
        {
            if (keyInput.Key == KeyCodes.F)
            {
                _joint2.EnableMotor(!_joint2.IsMotorEnabled());
                _joint2.BodyB.IsAwake = true;
            }

            if (keyInput.Key == KeyCodes.M)
            {
                _joint1.EnableMotor(!_joint1.IsMotorEnabled());
                _joint1.BodyB.IsAwake = true;
            }
        }

        protected override void OnRender()
        {
            DrawString("Keys: F toggle friction, M toggle motor");
            var torque = _joint1.GetMotorTorque(TestSettings.Hertz);
            DrawString($"Motor Torque = {torque}");
            DrawString($"Friction: {_joint2.IsMotorEnabled()}");
            DrawString($"Motor: {_joint1.IsMotorEnabled()}");
        }
    }
}