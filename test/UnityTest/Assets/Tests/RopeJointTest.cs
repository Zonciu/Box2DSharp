using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using UnityEngine;
using Joint = Box2DSharp.Dynamics.Joints.Joint;
using Vector2 = System.Numerics.Vector2;

namespace Box2DSharp.Tests
{
    [TestName("Rope Joint")]
    public class RopeJointTest : TestBase
    {
        private Joint _rope;

        private RopeJointDef _ropeDef = new RopeJointDef();

        protected override void Create()
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
                _ropeDef.LocalAnchorA.Set(0.0f, y);

                var prevBody = ground;
                for (var i = 0; i < N; ++i)
                {
                    var bd = new BodyDef();
                    bd.BodyType = BodyType.DynamicBody;
                    bd.Position.Set(0.5f + 1.0f * i, y);
                    if (i == N - 1)
                    {
                        shape.SetAsBox(1.5f, 1.5f);
                        fd.Density = 100.0f;
                        filter = fd.Filter;
                        filter.CategoryBits = 0x0002;
                        fd.Filter = filter;
                        bd.Position.Set(1.0f * i, y);
                        bd.AngularDamping = 0.4f;
                    }

                    var body = World.CreateBody(bd);

                    body.CreateFixture(fd);

                    var anchor = new Vector2(i, y);
                    jd.Initialize(prevBody, body, anchor);
                    World.CreateJoint(jd);

                    prevBody = body;
                }

                _ropeDef.LocalAnchorB.SetZero();

                var extraLength = 0.01f;
                _ropeDef.MaxLength = N - 1.0f + extraLength;
                _ropeDef.BodyB = prevBody;
            }

            {
                _ropeDef.BodyA = ground;
                _rope = World.CreateJoint(_ropeDef);
            }
        }

        /// <inheritdoc />
        protected override void PostStep()
        {
            if (Input.GetKeyDown(KeyCode.J))
            {
                if (_rope != null)
                {
                    World.DestroyJoint(_rope);
                    _rope = null;
                }
                else
                {
                    _rope = World.CreateJoint(_ropeDef);
                }
            }

            DrawString("Press (j) to toggle the rope joint.");
            if (_rope != null)
            {
                DrawString("Rope ON");
            }
            else
            {
                DrawString("Rope OFF");
            }
        }
    }
}