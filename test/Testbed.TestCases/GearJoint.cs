using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Joints", "Gear")]
    public class GearJoint : TestBase
    {
        private RevoluteJoint _joint1;

        private RevoluteJoint _joint2;

        private PrismaticJoint _joint3;

        private Box2DSharp.Dynamics.Joints.GearJoint _joint4;

        private Box2DSharp.Dynamics.Joints.GearJoint _joint5;

        public GearJoint()
        {
            Body ground;
            {
                var bd = new BodyDef();
                ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(50.0f, 0.0f), new Vector2(-50.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);
            }

            {
                var circle1 = new CircleShape();
                circle1.Radius = 1.0f;

                var box = new PolygonShape();
                box.SetAsBox(0.5f, 5.0f);

                var circle2 = new CircleShape();
                circle2.Radius = 2.0f;

                var bd1 = new BodyDef();
                bd1.BodyType = BodyType.StaticBody;
                bd1.Position.Set(10.0f, 9.0f);
                var body1 = World.CreateBody(bd1);
                body1.CreateFixture(circle1, 5.0f);

                var bd2 = new BodyDef();
                bd2.BodyType = BodyType.DynamicBody;
                bd2.Position.Set(10.0f, 8.0f);
                var body2 = World.CreateBody(bd2);
                body2.CreateFixture(box, 5.0f);

                var bd3 = new BodyDef();
                bd3.BodyType = BodyType.DynamicBody;
                bd3.Position.Set(10.0f, 6.0f);
                var body3 = World.CreateBody(bd3);
                body3.CreateFixture(circle2, 5.0f);

                var jd1 = new RevoluteJointDef();
                jd1.Initialize(body1, body2, bd1.Position);
                var joint1 = World.CreateJoint(jd1);

                var jd2 = new RevoluteJointDef();
                jd2.Initialize(body2, body3, bd3.Position);
                var joint2 = World.CreateJoint(jd2);

                var jd4 = new GearJointDef();
                jd4.BodyA = body1;
                jd4.BodyB = body3;
                jd4.Joint1 = joint1;
                jd4.Joint2 = joint2;
                jd4.Ratio = circle2.Radius / circle1.Radius;
                World.CreateJoint(jd4);
            }

            {
                var circle1 = new CircleShape();
                circle1.Radius = 1.0f;

                var circle2 = new CircleShape();
                circle2.Radius = 2.0f;

                var box = new PolygonShape();
                box.SetAsBox(0.5f, 5.0f);

                var bd1 = new BodyDef();
                bd1.BodyType = BodyType.DynamicBody;
                bd1.Position.Set(-3.0f, 12.0f);
                var body1 = World.CreateBody(bd1);
                body1.CreateFixture(circle1, 5.0f);

                var jd1 = new RevoluteJointDef();
                jd1.BodyA = ground;
                jd1.BodyB = body1;
                jd1.LocalAnchorA = ground.GetLocalPoint(bd1.Position);
                jd1.LocalAnchorB = body1.GetLocalPoint(bd1.Position);
                jd1.ReferenceAngle = body1.GetAngle() - ground.GetAngle();
                _joint1 = (RevoluteJoint)World.CreateJoint(jd1);

                var bd2 = new BodyDef();
                bd2.BodyType = BodyType.DynamicBody;
                bd2.Position.Set(0.0f, 12.0f);
                var body2 = World.CreateBody(bd2);
                body2.CreateFixture(circle2, 5.0f);

                var jd2 = new RevoluteJointDef();
                jd2.Initialize(ground, body2, bd2.Position);
                _joint2 = (RevoluteJoint)World.CreateJoint(jd2);

                var bd3 = new BodyDef();
                bd3.BodyType = BodyType.DynamicBody;
                bd3.Position.Set(2.5f, 12.0f);
                var body3 = World.CreateBody(bd3);
                body3.CreateFixture(box, 5.0f);

                var jd3 = new PrismaticJointDef();
                jd3.Initialize(ground, body3, bd3.Position, new Vector2(0.0f, 1.0f));
                jd3.LowerTranslation = -5.0f;
                jd3.UpperTranslation = 5.0f;
                jd3.EnableLimit = true;

                _joint3 = (PrismaticJoint)World.CreateJoint(jd3);

                var jd4 = new GearJointDef();
                jd4.BodyA = body1;
                jd4.BodyB = body2;
                jd4.Joint1 = _joint1;
                jd4.Joint2 = _joint2;
                jd4.Ratio = circle2.Radius / circle1.Radius;
                _joint4 = (Box2DSharp.Dynamics.Joints.GearJoint)World.CreateJoint(jd4);

                var jd5 = new GearJointDef();
                jd5.BodyA = body2;
                jd5.BodyB = body3;
                jd5.Joint1 = _joint2;
                jd5.Joint2 = _joint3;
                jd5.Ratio = -1.0f / circle2.Radius;
                _joint5 = (Box2DSharp.Dynamics.Joints.GearJoint)World.CreateJoint(jd5);
            }
        }

        protected override void OnRender()
        {
            var ratio = _joint4.GetRatio();
            var value = _joint1.GetJointAngle() + ratio * _joint2.GetJointAngle();
            DrawString($"theta1 + {ratio:F2} * theta2 ={value:F2}");

            ratio = _joint5.GetRatio();
            value = _joint2.GetJointAngle() + ratio * _joint3.GetJointTranslation();
            DrawString($"theta2 + {ratio:F2} * delta = {value:F2}");
        }
    }
}