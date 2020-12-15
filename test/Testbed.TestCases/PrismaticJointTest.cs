using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Testbed.Abstractions;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.TestCases
{
    /// <summary>
    /// Test the prismatic joint with limits and motor options.
    /// </summary>
    [TestCase("Joints", "Prismatic")]
    public class PrismaticJointTest : TestBase
    {
        protected PrismaticJoint Joint;

        protected float MotorSpeed;

        protected bool EnableMotor;

        protected bool EnableLimit;

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

            EnableLimit = true;
            EnableMotor = false;
            MotorSpeed = 10.0f;

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

                pjd.MotorSpeed = MotorSpeed;
                pjd.MaxMotorForce = 10000.0f;
                pjd.EnableMotor = EnableMotor;
                pjd.LowerTranslation = -10.0f;
                pjd.UpperTranslation = 10.0f;
                pjd.EnableLimit = EnableLimit;

                Joint = (PrismaticJoint)World.CreateJoint(pjd);
            }
        }

        /// <inheritdoc />
        public override void OnKeyDown(KeyInputEventArgs keyInput)
        {
            if (keyInput.Key == KeyCodes.L)
            {
                Joint.EnableLimit(!Joint.IsLimitEnabled());
            }

            if (keyInput.Key == KeyCodes.L)
            {
                Joint.EnableMotor(!Joint.IsMotorEnabled());
            }

            if (keyInput.Key == KeyCodes.L)
            {
                Joint.SetMotorSpeed(-Joint.GetMotorSpeed());
            }
        }

        /// <inheritdoc />
        protected override void OnRender()
        {
            var force = Joint.GetMotorForce(TestSettings.Hertz);
            DrawString($"Motor Force = {force}");
        }
    }
}