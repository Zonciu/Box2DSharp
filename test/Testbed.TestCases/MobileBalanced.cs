using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Solver", "Mobile Balanced")]
    public class MobileBalanced : TestBase
    {
        private const int e_depth = 4;

        public MobileBalanced()
        {
            Body ground;

            // Create ground body.
            {
                var bodyDef = new BodyDef();
                bodyDef.Position.Set(0.0f, 20.0f);
                ground = World.CreateBody(bodyDef);
            }

            var a = 0.5f;
            var h = new Vector2(0.0f, a);

            var root = AddNode(ground, Vector2.Zero, 0, 3.0f, a);

            var jointDef = new RevoluteJointDef();
            jointDef.BodyA = ground;
            jointDef.BodyB = root;
            jointDef.LocalAnchorA.SetZero();
            jointDef.LocalAnchorB = h;
            World.CreateJoint(jointDef);
        }

        private Body AddNode(Body parent, Vector2 localAnchor, int depth, float offset, float a)
        {
            var density = 20.0f;
            var h = new Vector2(0.0f, a);

            var p = parent.GetPosition() + localAnchor - h;

            var bodyDef = new BodyDef();
            bodyDef.BodyType = BodyType.DynamicBody;
            bodyDef.Position = p;
            var body = World.CreateBody(bodyDef);

            var shape = new PolygonShape();
            shape.SetAsBox(0.25f * a, a);
            body.CreateFixture(shape, density);

            if (depth == e_depth)
            {
                return body;
            }

            shape.SetAsBox(offset, 0.25f * a, new Vector2(0, -a), 0.0f);
            body.CreateFixture(shape, density);

            var a1 = new Vector2(offset, -a);
            var a2 = new Vector2(-offset, -a);
            var body1 = AddNode(body, a1, depth + 1, 0.5f * offset, a);
            var body2 = AddNode(body, a2, depth + 1, 0.5f * offset, a);

            var jointDef = new RevoluteJointDef();
            jointDef.BodyA = body;
            jointDef.LocalAnchorB = h;

            jointDef.LocalAnchorA = a1;
            jointDef.BodyB = body1;
            World.CreateJoint(jointDef);

            jointDef.LocalAnchorA = a2;
            jointDef.BodyB = body2;
            World.CreateJoint(jointDef);

            return body;
        }
    }
}