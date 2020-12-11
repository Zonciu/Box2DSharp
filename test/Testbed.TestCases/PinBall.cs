using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Testbed.Abstractions;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.TestCases
{
    [TestCase("Examples", "Pinball")]
    public class PinBall : TestBase
    {
        private Body _ball;

        public bool _button;

        private RevoluteJoint _leftJoint;

        private RevoluteJoint _rightJoint;

        public PinBall()
        {
            // Ground body
            Body ground;
            {
                var bd = new BodyDef();
                ground = World.CreateBody(bd);

                var vs = new Vector2[5];
                vs[0].Set(-8.0f, 6.0f);
                vs[1].Set(-8.0f, 20.0f);
                vs[2].Set(8.0f, 20.0f);
                vs[3].Set(8.0f, 6.0f);
                vs[4].Set(0.0f, -2.0f);

                var loop = new ChainShape();
                loop.CreateLoop(vs);
                var fd = new FixtureDef();
                fd.Shape = loop;
                fd.Density = 0.0f;
                ground.CreateFixture(fd);
            }

            // Flippers
            {
                var p1 = new Vector2(-2.0f, 0.0f);
                var p2 = new Vector2(2.0f, 0.0f);

                var bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;

                bd.Position = p1;
                var leftFlipper = World.CreateBody(bd);

                bd.Position = p2;
                var rightFlipper = World.CreateBody(bd);

                var box = new PolygonShape();
                box.SetAsBox(1.75f, 0.1f);

                var fd = new FixtureDef();
                fd.Shape = box;
                fd.Density = 1.0f;

                leftFlipper.CreateFixture(fd);
                rightFlipper.CreateFixture(fd);

                var jd = new RevoluteJointDef();
                jd.BodyA = ground;
                jd.LocalAnchorB.SetZero();
                jd.EnableMotor = true;
                jd.MaxMotorTorque = 1000.0f;
                jd.EnableLimit = true;

                jd.MotorSpeed = 0.0f;
                jd.LocalAnchorA = p1;
                jd.BodyB = leftFlipper;
                jd.LowerAngle = -30.0f * Settings.Pi / 180.0f;
                jd.UpperAngle = 5.0f * Settings.Pi / 180.0f;
                _leftJoint = (RevoluteJoint)World.CreateJoint(jd);

                jd.MotorSpeed = 0.0f;
                jd.LocalAnchorA = p2;
                jd.BodyB = rightFlipper;
                jd.LowerAngle = -5.0f * Settings.Pi / 180.0f;
                jd.UpperAngle = 30.0f * Settings.Pi / 180.0f;
                _rightJoint = (RevoluteJoint)World.CreateJoint(jd);
            }

            // Circle character
            {
                var bd = new BodyDef();
                bd.Position.Set(1.0f, 15.0f);
                bd.BodyType = BodyType.DynamicBody;
                bd.Bullet = true;

                _ball = World.CreateBody(bd);

                var shape = new CircleShape();
                shape.Radius = 0.2f;

                var fd = new FixtureDef();
                fd.Shape = shape;
                fd.Density = 1.0f;
                _ball.CreateFixture(fd);
            }

            _button = false;
        }

        /// <inheritdoc />
        public override void OnKeyDown(KeyInputEventArgs keyInput)
        {
            if (keyInput.Key == KeyCodes.A)
            {
                _button = true;
            }
        }

        /// <inheritdoc />
        public override void OnKeyUp(KeyInputEventArgs keyInput)
        {
            if (keyInput.Key == KeyCodes.A)
            {
                _button = false;
            }
        }

        protected override void PreStep()
        {
            if (_button)
            {
                _leftJoint.SetMotorSpeed(20.0f);
                _rightJoint.SetMotorSpeed(-20.0f);
            }
            else
            {
                _leftJoint.SetMotorSpeed(-10.0f);
                _rightJoint.SetMotorSpeed(10.0f);
            }
        }

        protected override void OnRender()
        {
            DrawString("Press 'a' to control the flippers");
        }
    }
}