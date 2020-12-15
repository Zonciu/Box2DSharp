using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Testbed.Abstractions;
using Joint = Box2DSharp.Dynamics.Joints.Joint;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.TestCases
{
    /// This test shows how a distance joint can be used to stabilize a chain of
    /// bodies with a heavy payload. Notice that the distance joint just prevents
    /// excessive stretching and has no other effect.
    /// By disabling the distance joint you can see that the Box2D solver has trouble
    /// supporting heavy bodies with light bodies. Try playing around with the
    /// densities, time step, and iterations to see how they affect stability.
    /// This test also shows how to use contact filtering. Filtering is configured
    /// so that the payload does not collide with the chain.
    [TestCase("Examples", "Wrecking Ball")]
    public class WreckingBall : TestBase
    {
        private Joint _rope;

        protected DistanceJointDef _distanceJointDef = new DistanceJointDef();

        protected Joint _distanceJoint;

        protected bool _stabilize;

        public WreckingBall()
        {
            Body ground;
            {
                var bd = new BodyDef();
                ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);
            }

            {
                var shape = new PolygonShape();
                shape.SetAsBox(0.5f, 0.125f);

                var fd = new FixtureDef();
                fd.Shape = shape;
                fd.Density = 20.0f;
                fd.Friction = 0.2f;
                var filter = fd.Filter;
                filter.CategoryBits = 0x0001;
                filter.MaskBits = 0xFFFF & ~0x0002;
                fd.Filter = filter;
                var jd = new RevoluteJointDef();
                jd.CollideConnected = false;

                const int N = 10;
                const float y = 15.0f;
                _distanceJointDef.LocalAnchorA.Set(0.0f, y);

                var prevBody = ground;
                for (var i = 0; i < N; ++i)
                {
                    var bd = new BodyDef();
                    bd.BodyType = BodyType.DynamicBody;
                    bd.Position.Set(0.5f + 1.0f * i, y);
                    if (i == N - 1)
                    {
                        bd.Position.Set(1.0f * i, y);
                        bd.AngularDamping = 0.4f;
                    }

                    var body = World.CreateBody(bd);
                    if (i == N - 1)
                    {
                        CircleShape circleShape = new CircleShape();
                        circleShape.Radius = 1.5f;
                        FixtureDef sfd = new FixtureDef();
                        sfd.Shape = circleShape;
                        sfd.Density = 100.0f;
                        sfd.Filter.CategoryBits = 0x0002;
                        body.CreateFixture(sfd);
                    }
                    else
                    {
                        body.CreateFixture(fd);
                    }

                    body.CreateFixture(fd);

                    var anchor = new Vector2(i, y);
                    jd.Initialize(prevBody, body, anchor);
                    World.CreateJoint(jd);

                    prevBody = body;
                }

                _distanceJointDef.LocalAnchorB.SetZero();

                var extraLength = 0.01f;
                _distanceJointDef.MinLength = 0.0f;
                _distanceJointDef.MaxLength = N - 1.0f + extraLength;
                _distanceJointDef.BodyB = prevBody;
            }

            {
                _distanceJointDef.BodyA = ground;
                _distanceJoint = World.CreateJoint(_distanceJointDef);
                _stabilize = true;
            }
        }

        /// <inheritdoc />
        /// <inheritdoc />
        public override void OnKeyDown(KeyInputEventArgs keyInput)
        {
            if (keyInput.Key == KeyCodes.J)
            {
                if (_rope != null)
                {
                    World.DestroyJoint(_rope);
                    _rope = null;
                }
                else
                {
                    _rope = World.CreateJoint(_distanceJointDef);
                }
            }
        }

        protected override void OnRender()
        {
            if (_distanceJoint != null)
            {
                DrawString("Distance Joint ON");
            }
            else
            {
                DrawString("Distance Joint OFF");
            }
        }
    }
}