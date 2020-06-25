using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using ImGuiNET;
using Testbed.Basics;

namespace Testbed.Tests
{
    [TestCase("Joints", "Wheel")]
    public class WheelJointTest : Test
    {
        private WheelJoint _joint;

        private float _motorSpeed;

        private bool _enableMotor;

        private bool _enableLimit;

        public WheelJointTest()
        {
            Body ground = null;
            {
                var bd = new BodyDef();
                ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.Set(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);
            }

            _enableLimit = true;
            _enableMotor = false;
            _motorSpeed = 10.0f;

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

                jd.MotorSpeed = _motorSpeed;
                jd.MaxMotorTorque = 10000.0f;
                jd.EnableMotor = _enableMotor;
                jd.Stiffness = mass * omega * omega;
                jd.Damping = 2.0f * mass * dampingRatio * omega;
                jd.LowerTranslation = -3.0f;
                jd.UpperTranslation = 3.0f;
                jd.EnableLimit = _enableLimit;

                _joint = (WheelJoint)World.CreateJoint(jd);
            }
        }

        /// <inheritdoc />
        protected override void OnRender()
        {
            var torque = _joint.GetMotorTorque(TestSettings.Hertz);
            DrawString($"Motor Torque = {torque}");

            ImGui.SetNextWindowPos(new Vector2(10.0f, 100.0f));
            ImGui.SetNextWindowSize(new Vector2(200.0f, 100.0f));
            ImGui.Begin("Joint Controls", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);

            if (ImGui.Checkbox("Limit", ref _enableLimit))
            {
                _joint.EnableLimit(_enableLimit);
            }

            if (ImGui.Checkbox("Motor", ref _enableMotor))
            {
                _joint.EnableMotor(_enableMotor);
            }

            if (ImGui.SliderFloat("Speed", ref _motorSpeed, -100.0f, 100.0f, "%.0f"))
            {
                _joint.SetMotorSpeed(_motorSpeed);
            }

            ImGui.End();
        }
    }
}