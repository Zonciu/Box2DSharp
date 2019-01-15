using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;

namespace Box2DSharp.Tests
{
    public class Pulleys : TestBase
    {
        private PulleyJoint _joint1;

        protected override void Create()
        {
            var y = 16.0f;
            var L = 12.0f;
            var a = 1.0f;
            var b = 2.0f;

            Body ground;
            {
                var bd = new BodyDef();
                ground = World.CreateBody(bd);

                var edge = new EdgeShape();
                edge.Set(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));

                //ground.CreateFixture(shape, 0.0f);

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

                _joint1 = (PulleyJoint) World.CreateJoint(pulleyDef);
            }
        }

        /// <inheritdoc />
        protected override void PreStep()
        {
            var ratio = _joint1.GetRatio();
            var L = _joint1.GetCurrentLengthA() + ratio * _joint1.GetCurrentLengthB();
            DrawString($"L1 + {ratio:F2} * L2 = {L:F2}");
        }
    }
}