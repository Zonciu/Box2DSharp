using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using OpenToolkit.Windowing.Common;
using OpenToolkit.Windowing.Common.Input;
using Testbed.Basics;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.Tests
{
    [TestCase("Joints", "Prismatic")]
    public class PrismaticJoint : Test
    {
        private Box2DSharp.Dynamics.Joints.PrismaticJoint _joint;

        public PrismaticJoint()
        {
            Body ground;
            {
                var bd = new BodyDef();
                ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.Set(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);
            }

            {
                var shape = new PolygonShape();
                shape.SetAsBox(2.0f, 0.5f);

                var bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(-10.0f, 10.0f);
                bd.Angle = 0.5f * Settings.Pi;
                bd.AllowSleep = false;
                var body = World.CreateBody(bd);
                body.CreateFixture(shape, 5.0f);

                var pjd = new PrismaticJointDef();

                // Bouncy limit
                var axis = new Vector2(2.0f, 1.0f);

                axis = Vector2.Normalize(axis);
                pjd.Initialize(ground, body, new Vector2(0.0f, 0.0f), axis);

                // Non-bouncy limit
                //pjd.Initialize(ground, body, new Vector2(-10.0f, 10.0f), new Vector2(1.0f, 0.0f));

                pjd.MotorSpeed = 10.0f;
                pjd.MaxMotorForce = 10000.0f;
                pjd.EnableMotor = true;
                pjd.LowerTranslation = 0.0f;
                pjd.UpperTranslation = 20.0f;
                pjd.EnableLimit = true;

                _joint = (Box2DSharp.Dynamics.Joints.PrismaticJoint)World.CreateJoint(pjd);
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
            DrawString("Keys: (l) limits, (m) motors, (s) speed");

            var force = _joint.GetMotorForce(TestSettings.Hertz);
            DrawString($"Motor Force = {force}");
        }
    }
}