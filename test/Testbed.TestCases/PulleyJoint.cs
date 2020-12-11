using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Joints", "Pulley")]
    public class PulleyJoint : TestBase
    {
        private Box2DSharp.Dynamics.Joints.PulleyJoint _joint1;

        public PulleyJoint()
        {
            var y = 16.0f;
            var L = 12.0f;
            var a = 1.0f;
            var b = 2.0f;

            Body ground;
            {
                var bd = new BodyDef();
                ground = World.CreateBody(bd);

                var circle = new CircleShape();
                circle.Radius = 2.0f;

                circle.Position.Set(-10.0f, y + b + L);
                ground.CreateFixture(circle, 0.0f);

                circle.Position.Set(10.0f, y + b + L);
                ground.CreateFixture(circle, 0.0f);
            }

            {
                var shape = new PolygonShape();
                shape.SetAsBox(a, b);

                var bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;

                //bd.fixedRotation = true;
                bd.Position.Set(-10.0f, y);
                var body1 = World.CreateBody(bd);
                body1.CreateFixture(shape, 5.0f);

                bd.Position.Set(10.0f, y);
                var body2 = World.CreateBody(bd);
                body2.CreateFixture(shape, 5.0f);

                var pulleyDef = new PulleyJointDef();
                var anchor1 = new Vector2(-10.0f, y + b);
                var anchor2 = new Vector2(10.0f, y + b);
                var groundAnchor1 = new Vector2(-10.0f, y + b + L);
                var groundAnchor2 = new Vector2(10.0f, y + b + L);
                pulleyDef.Initialize(
                    body1,
                    body2,
                    groundAnchor1,
                    groundAnchor2,
                    anchor1,
                    anchor2,
                    1.5f);

                _joint1 = (Box2DSharp.Dynamics.Joints.PulleyJoint)World.CreateJoint(pulleyDef);
            }
        }

        /// <inheritdoc />
        protected override void OnRender()
        {
            var ratio = _joint1.GetRatio();
            var L = _joint1.GetCurrentLengthA() + ratio * _joint1.GetCurrentLengthB();
            DrawString($"L1 + {ratio:F2} * L2 = {L:F2}");
        }
    }
}