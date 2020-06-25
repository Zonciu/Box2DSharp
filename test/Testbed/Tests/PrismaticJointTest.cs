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
    [TestCase("Joints", "Prismatic")]
    public class PrismaticJointTest : Test
    {
        private PrismaticJoint _joint;

        float m_motorSpeed;

        bool m_enableMotor;

        bool m_enableLimit;

        public PrismaticJointTest()
        {
            Body ground;
            {
                var bd = new BodyDef();
                ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.Set(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);
            }

            m_enableLimit = true;
            m_enableMotor = false;
            m_motorSpeed = 10.0f;

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

                pjd.MotorSpeed = m_motorSpeed;
                pjd.MaxMotorForce = 10000.0f;
                pjd.EnableMotor = m_enableMotor;
                pjd.LowerTranslation = -10.0f;
                pjd.UpperTranslation = 10.0f;
                pjd.EnableLimit = m_enableLimit;

                _joint = (PrismaticJoint)World.CreateJoint(pjd);
            }

            // {
            //     var shape = new PolygonShape();
            //     shape.SetAsBox(2.0f, 0.5f);
            //
            //     var bd = new BodyDef();
            //     bd.BodyType = BodyType.DynamicBody;
            //     bd.Position.Set(-10.0f, 10.0f);
            //     bd.Angle = 0.5f * Settings.Pi;
            //     bd.AllowSleep = false;
            //     var body = World.CreateBody(bd);
            //     body.CreateFixture(shape, 5.0f);
            //
            //     var pjd = new PrismaticJointDef();
            //
            //     // Bouncy limit
            //     var axis = new Vector2(2.0f, 1.0f);
            //
            //     axis = Vector2.Normalize(axis);
            //     pjd.Initialize(ground, body, new Vector2(0.0f, 0.0f), axis);
            //
            //     // Non-bouncy limit
            //     //pjd.Initialize(ground, body, new Vector2(-10.0f, 10.0f), new Vector2(1.0f, 0.0f));
            //
            //     pjd.MotorSpeed = 10.0f;
            //     pjd.MaxMotorForce = 10000.0f;
            //     pjd.EnableMotor = true;
            //     pjd.LowerTranslation = 0.0f;
            //     pjd.UpperTranslation = 20.0f;
            //     pjd.EnableLimit = true;
            //
            //     _joint = (PrismaticJoint)World.CreateJoint(pjd);
            // }
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

            if (ImGui.Checkbox("Limit", ref m_enableLimit))
            {
                _joint.EnableLimit(m_enableLimit);
            }

            if (ImGui.Checkbox("Motor", ref m_enableMotor))
            {
                _joint.EnableMotor(m_enableMotor);
            }

            if (ImGui.SliderFloat("Speed", ref m_motorSpeed, -100.0f, 100.0f, "%.0f"))
            {
                _joint.SetMotorSpeed(m_motorSpeed);
            }

            ImGui.End();
        }
    }
}