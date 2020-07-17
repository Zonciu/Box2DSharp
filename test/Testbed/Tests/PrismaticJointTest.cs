using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using ImGuiNET;
using OpenToolkit.Windowing.Common;
using OpenToolkit.Windowing.Common.Input;
using Testbed.Basics;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.Tests
{
    /// <summary>
    /// Test the prismatic joint with limits and motor options.
    /// </summary>
    [TestCase("Joints", "Prismatic")]
    public class PrismaticJointTest : Test
    {
        private PrismaticJoint _joint;

        private float _motorSpeed;

        private bool _enableMotor;

        private bool _enableLimit;

        public PrismaticJointTest()
        {
            Body ground;
            {
                var bd = new BodyDef();
                ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);
            }

            _enableLimit = true;
            _enableMotor = false;
            _motorSpeed = 10.0f;

            {
                PolygonShape shape = new PolygonShape();
                shape.SetAsBox(1.0f, 1.0f);

                BodyDef bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(0.0f, 10.0f);
                bd.Angle = 0.5f * Settings.Pi;
                bd.AllowSleep = false;
                var body = World.CreateBody(bd);
                body.CreateFixture(shape, 5.0f);

                PrismaticJointDef pjd = new PrismaticJointDef();

                // Horizontal
                pjd.Initialize(ground, body, bd.Position, new Vector2(1.0f, 0.0f));

                pjd.MotorSpeed = _motorSpeed;
                pjd.MaxMotorForce = 10000.0f;
                pjd.EnableMotor = _enableMotor;
                pjd.LowerTranslation = -10.0f;
                pjd.UpperTranslation = 10.0f;
                pjd.EnableLimit = _enableLimit;

                _joint = (PrismaticJoint)World.CreateJoint(pjd);
            }
        }

        /// <inheritdoc />
        public override void OnKeyDown(KeyboardKeyEventArgs key)
        {
            if (key.Key == Key.L)
            {
                _joint.EnableLimit(!_joint.IsLimitEnabled());
            }

            if (key.Key == Key.L)
            {
                _joint.EnableMotor(!_joint.IsMotorEnabled());
            }

            if (key.Key == Key.L)
            {
                _joint.SetMotorSpeed(-_joint.GetMotorSpeed());
            }
        }

        /// <inheritdoc />
        protected override void OnRender()
        {
            var force = _joint.GetMotorForce(TestSettings.Hertz);
            DrawString($"Motor Force = {force}");
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