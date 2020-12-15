using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Testbed.Abstractions;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.TestCases
{
    [TestCase("Joints", "Revolute")]
    public class RevoluteJointTest : TestBase
    {
        private Body _ball;

        protected RevoluteJoint Joint1;

        protected RevoluteJoint Joint2;

        protected bool EnableLimit = true;

        protected bool EnableMotor = false;

        protected float MotorSpeed = 1.0f;

        public RevoluteJointTest()
        {
            Body ground;
            {
                var bd = new BodyDef();
                ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));

                var fd = new FixtureDef();
                fd.Shape = shape;

                //fd.filter.categoryBits = 2;

                ground.CreateFixture(fd);
            }

            EnableLimit = true;
            EnableMotor = false;
            MotorSpeed = 1.0f;

            {
                PolygonShape shape = new PolygonShape();
                shape.SetAsBox(0.25f, 3.0f, new Vector2(0.0f, 3.0f), 0.0f);

                BodyDef bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(-10.0f, 20.0f);
                Body body = World.CreateBody(bd);
                body.CreateFixture(shape, 5.0f);

                RevoluteJointDef jd = new RevoluteJointDef();
                jd.Initialize(ground, body, new Vector2(-10.0f, 20.5f));
                jd.MotorSpeed = MotorSpeed;
                jd.MaxMotorTorque = 10000.0f;
                jd.EnableMotor = EnableMotor;
                jd.LowerAngle = -0.25f * Settings.Pi;
                jd.UpperAngle = 0.5f * Settings.Pi;
                jd.EnableLimit = EnableLimit;

                Joint1 = (RevoluteJoint)World.CreateJoint(jd);
            }

            {
                CircleShape circle_shape = new CircleShape();
                circle_shape.Radius = 2.0f;

                BodyDef circle_bd = new BodyDef();
                circle_bd.BodyType = BodyType.DynamicBody;
                circle_bd.Position.Set(5.0f, 30.0f);

                FixtureDef fd = new FixtureDef();
                fd.Density = 5.0f;
                fd.Filter.MaskBits = 1;
                fd.Shape = circle_shape;

                _ball = World.CreateBody(circle_bd);
                _ball.CreateFixture(fd);

                PolygonShape polygon_shape = new PolygonShape();
                polygon_shape.SetAsBox(10.0f, 0.5f, new Vector2(-10.0f, 0.0f), 0.0f);

                BodyDef polygon_bd = circle_bd;
                polygon_bd.Position.Set(20.0f, 10.0f);
                polygon_bd.BodyType = BodyType.DynamicBody;
                polygon_bd.Bullet = true;
                var polygon_body = World.CreateBody(polygon_bd);
                polygon_body.CreateFixture(polygon_shape, 2.0f);

                RevoluteJointDef jd = new RevoluteJointDef();
                jd.Initialize(ground, polygon_body, new Vector2(19.0f, 10.0f));
                jd.LowerAngle = -0.25f * Settings.Pi;
                jd.UpperAngle = 0.0f * Settings.Pi;
                jd.EnableLimit = true;
                jd.EnableMotor = true;
                jd.MotorSpeed = 0.0f;
                jd.MaxMotorTorque = 10000.0f;

                Joint2 = (RevoluteJoint)World.CreateJoint(jd);
            }
        }

        /// <inheritdoc />
        protected override void PreStep()
        {
            var torque1 = Joint1.GetMotorTorque(TestSettings.Hertz);
            DrawString($"Motor Torque 1= {torque1}");

            var torque2 = Joint2.GetMotorTorque(TestSettings.Hertz);
            DrawString($"Motor Torque 2= {torque2}");
        }

        protected override void OnRender()
        {
            DrawString("Keys: (l) limits, (m) motor");
        }
    }
}