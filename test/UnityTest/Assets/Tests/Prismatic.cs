using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using UnityEngine;
using Vector2 = System.Numerics.Vector2;

namespace Box2DSharp.Tests
{
    public class Prismatic : TestBase
    {
        private PrismaticJoint _joint;

        private void Start()
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

                _joint = (PrismaticJoint) World.CreateJoint(pjd);
            }
        }

        /// <inheritdoc />
        protected override void PreStep()
        {
            DrawString("Keys: (l) limits, (m) motors, (s) speed");

            var force = _joint.GetMotorForce(TestSettings.Frequency);
            DrawString($"Motor Force = {force}");

            if (Input.GetKeyDown(KeyCode.L))
            {
                _joint.EnableLimit(!_joint.IsLimitEnabled());
            }

            if (Input.GetKeyDown(KeyCode.L))
            {
                _joint.EnableMotor(!_joint.IsMotorEnabled());
            }

            if (Input.GetKeyDown(KeyCode.L))
            {
                _joint.SetMotorSpeed(-_joint.GetMotorSpeed());
            }
        }
    }
}