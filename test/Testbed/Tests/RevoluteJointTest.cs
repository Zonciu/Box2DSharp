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
    [TestCase("Joints", "Revolute")]
    public class RevoluteJointTest : Test
    {
        private Body m_ball;

        private RevoluteJoint m_joint;

        public RevoluteJointTest()
        {
            Body ground;
            {
                var bd = new BodyDef();
                ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.Set(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));

                var fd = new FixtureDef();
                fd.Shape = shape;

                //fd.filter.categoryBits = 2;

                ground.CreateFixture(fd);
            }

            {
                var shape = new CircleShape();
                shape.Radius = 0.5f;

                var bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;

                var rjd = new RevoluteJointDef();

                bd.Position.Set(-10.0f, 20.0f);
                var body = World.CreateBody(bd);
                body.CreateFixture(shape, 5.0f);

                var w = 100.0f;
                body.SetAngularVelocity(w);
                body.SetLinearVelocity(new Vector2(-8.0f * w, 0.0f));

                rjd.Initialize(ground, body, new Vector2(-10.0f, 12.0f));
                rjd.MotorSpeed = 1.0f * Settings.Pi;
                rjd.MaxMotorTorque = 10000.0f;
                rjd.EnableMotor = false;
                rjd.LowerAngle = -0.25f * Settings.Pi;
                rjd.UpperAngle = 0.5f * Settings.Pi;
                rjd.EnableLimit = true;
                rjd.CollideConnected = true;

                m_joint = (RevoluteJoint)World.CreateJoint(rjd);
            }

            {
                var circle_shape = new CircleShape();
                circle_shape.Radius = 3.0f;

                var circle_bd = new BodyDef();
                circle_bd.BodyType = BodyType.DynamicBody;
                circle_bd.Position.Set(5.0f, 30.0f);

                var fd = new FixtureDef();
                fd.Density = 5.0f;
                var filter = fd.Filter;
                filter.MaskBits = 1;
                fd.Filter = filter;
                fd.Shape = circle_shape;

                m_ball = World.CreateBody(circle_bd);
                m_ball.CreateFixture(fd);

                var polygon_shape = new PolygonShape();
                polygon_shape.SetAsBox(10.0f, 0.2f, new Vector2(-10.0f, 0.0f), 0.0f);

                var polygon_bd = new BodyDef();
                polygon_bd.Position.Set(20.0f, 10.0f);
                polygon_bd.BodyType = BodyType.DynamicBody;
                polygon_bd.Bullet = true;
                var polygon_body = World.CreateBody(polygon_bd);
                polygon_body.CreateFixture(polygon_shape, 2.0f);

                var rjd = new RevoluteJointDef();
                rjd.Initialize(ground, polygon_body, new Vector2(20.0f, 10.0f));
                rjd.LowerAngle = -0.25f * Settings.Pi;
                rjd.UpperAngle = 0.0f * Settings.Pi;
                rjd.EnableLimit = true;
                World.CreateJoint(rjd);
            }

            // Tests mass computation of a small object far from the origin
            {
                var bodyDef = new BodyDef();
                bodyDef.BodyType = BodyType.DynamicBody;
                var body = World.CreateBody(bodyDef);

                var polyShape = new PolygonShape();
                var verts = new Vector2[3];
                verts[0].Set(17.63f, 36.31f);
                verts[1].Set(17.52f, 36.69f);
                verts[2].Set(17.19f, 36.36f);
                polyShape.Set(verts);

                var polyFixtureDef = new FixtureDef();
                polyFixtureDef.Shape = polyShape;
                polyFixtureDef.Density = 1;

                body.CreateFixture(polyFixtureDef); //assertion hits inside here
            }
        }

        /// <inheritdoc />
        /// <inheritdoc />
        public override void OnKeyDown(KeyboardKeyEventArgs key)
        {
            if (key.Key == Key.L)
            {
                m_joint.EnableLimit(!m_joint.IsLimitEnabled());
            }

            if (key.Key == Key.M)
            {
                m_joint.EnableMotor(!m_joint.IsMotorEnabled());
            }
        }

        protected override void OnRender()
        {
            DrawString("Keys: (l) limits, (m) motor");
        }
    }
}