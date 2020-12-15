using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Joints", "Wheel")]
    public class WheelJointTest : TestBase
    {
        protected WheelJoint Joint;

        protected float MotorSpeed;

        protected bool EnableMotor;

        protected bool EnableLimit;

        public WheelJointTest()
        {
            Body ground = null;
            {
                var bd = new BodyDef();
                ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);
            }

            EnableLimit = true;
            EnableMotor = false;
            MotorSpeed = 10.0f;

            {
                CircleShape shape = new CircleShape();
                shape.Radius = 2.0f;

                BodyDef bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(0.0f, 10.0f);
                bd.AllowSleep = false;
                var body = World.CreateBody(bd);
                body.CreateFixture(shape, 5.0f);

                var mass = body.Mass;
                var hertz = 1.0f;
                var dampingRatio = 0.7f;
                var omega = 2.0f * Settings.Pi * hertz;

                var jd = new WheelJointDef();

                // Horizontal
                jd.Initialize(ground, body, bd.Position, new Vector2(0.0f, 1.0f));

                jd.MotorSpeed = MotorSpeed;
                jd.MaxMotorTorque = 10000.0f;
                jd.EnableMotor = EnableMotor;
                jd.Stiffness = mass * omega * omega;
                jd.Damping = 2.0f * mass * dampingRatio * omega;
                jd.LowerTranslation = -3.0f;
                jd.UpperTranslation = 3.0f;
                jd.EnableLimit = EnableLimit;

                Joint = (WheelJoint)World.CreateJoint(jd);
            }
        }

        /// <inheritdoc />
        protected override void OnRender()
        {
            var torque = Joint.GetMotorTorque(TestSettings.Hertz);
            DrawString($"Motor Torque = {torque}");

            var F = Joint.GetReactionForce(TestSettings.Hertz);
            DrawString($"Reaction Force = ({F.X}, {F.X})");
        }
    }
}