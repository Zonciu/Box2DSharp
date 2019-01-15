using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using UnityEngine;
using Joint = Box2DSharp.Dynamics.Joints.Joint;
using Vector2 = System.Numerics.Vector2;

namespace Box2DSharp.Tests
{
    public class Web : TestBase
    {
        private readonly Body[] _bodies = new Body[4];

        private readonly Joint[] _joints = new Joint[8];

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
                shape.SetAsBox(0.5f, 0.5f);

                var bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;

                bd.Position.Set(-5.0f, 5.0f);
                _bodies[0] = World.CreateBody(bd);
                _bodies[0].CreateFixture(shape, 5.0f);

                bd.Position.Set(5.0f, 5.0f);
                _bodies[1] = World.CreateBody(bd);
                _bodies[1].CreateFixture(shape, 5.0f);

                bd.Position.Set(5.0f, 15.0f);
                _bodies[2] = World.CreateBody(bd);
                _bodies[2].CreateFixture(shape, 5.0f);

                bd.Position.Set(-5.0f, 15.0f);
                _bodies[3] = World.CreateBody(bd);
                _bodies[3].CreateFixture(shape, 5.0f);

                var jd = new DistanceJointDef();
                Vector2 p1;
                Vector2 p2;
                Vector2 d;

                jd.FrequencyHz = 2.0f;
                jd.DampingRatio = 0.0f;

                jd.BodyA = ground;
                jd.BodyB = _bodies[0];
                jd.LocalAnchorA.Set(-10.0f, 0.0f);
                jd.LocalAnchorB.Set(-0.5f, -0.5f);
                p1 = jd.BodyA.GetWorldPoint(jd.LocalAnchorA);
                p2 = jd.BodyB.GetWorldPoint(jd.LocalAnchorB);
                d = p2 - p1;
                jd.Length = d.Length();
                _joints[0] = World.CreateJoint(jd);

                jd.BodyA = ground;
                jd.BodyB = _bodies[1];
                jd.LocalAnchorA.Set(10.0f, 0.0f);
                jd.LocalAnchorB.Set(0.5f, -0.5f);
                p1 = jd.BodyA.GetWorldPoint(jd.LocalAnchorA);
                p2 = jd.BodyB.GetWorldPoint(jd.LocalAnchorB);
                d = p2 - p1;
                jd.Length = d.Length();
                _joints[1] = World.CreateJoint(jd);

                jd.BodyA = ground;
                jd.BodyB = _bodies[2];
                jd.LocalAnchorA.Set(10.0f, 20.0f);
                jd.LocalAnchorB.Set(0.5f, 0.5f);
                p1 = jd.BodyA.GetWorldPoint(jd.LocalAnchorA);
                p2 = jd.BodyB.GetWorldPoint(jd.LocalAnchorB);
                d = p2 - p1;
                jd.Length = d.Length();
                _joints[2] = World.CreateJoint(jd);

                jd.BodyA = ground;
                jd.BodyB = _bodies[3];
                jd.LocalAnchorA.Set(-10.0f, 20.0f);
                jd.LocalAnchorB.Set(-0.5f, 0.5f);
                p1 = jd.BodyA.GetWorldPoint(jd.LocalAnchorA);
                p2 = jd.BodyB.GetWorldPoint(jd.LocalAnchorB);
                d = p2 - p1;
                jd.Length = d.Length();
                _joints[3] = World.CreateJoint(jd);

                jd.BodyA = _bodies[0];
                jd.BodyB = _bodies[1];
                jd.LocalAnchorA.Set(0.5f, 0.0f);
                jd.LocalAnchorB.Set(-0.5f, 0.0f);
                ;
                p1 = jd.BodyA.GetWorldPoint(jd.LocalAnchorA);
                p2 = jd.BodyB.GetWorldPoint(jd.LocalAnchorB);
                d = p2 - p1;
                jd.Length = d.Length();
                _joints[4] = World.CreateJoint(jd);

                jd.BodyA = _bodies[1];
                jd.BodyB = _bodies[2];
                jd.LocalAnchorA.Set(0.0f, 0.5f);
                jd.LocalAnchorB.Set(0.0f, -0.5f);
                p1 = jd.BodyA.GetWorldPoint(jd.LocalAnchorA);
                p2 = jd.BodyB.GetWorldPoint(jd.LocalAnchorB);
                d = p2 - p1;
                jd.Length = d.Length();
                _joints[5] = World.CreateJoint(jd);

                jd.BodyA = _bodies[2];
                jd.BodyB = _bodies[3];
                jd.LocalAnchorA.Set(-0.5f, 0.0f);
                jd.LocalAnchorB.Set(0.5f, 0.0f);
                p1 = jd.BodyA.GetWorldPoint(jd.LocalAnchorA);
                p2 = jd.BodyB.GetWorldPoint(jd.LocalAnchorB);
                d = p2 - p1;
                jd.Length = d.Length();
                _joints[6] = World.CreateJoint(jd);

                jd.BodyA = _bodies[3];
                jd.BodyB = _bodies[0];
                jd.LocalAnchorA.Set(0.0f, -0.5f);
                jd.LocalAnchorB.Set(0.0f, 0.5f);
                p1 = jd.BodyA.GetWorldPoint(jd.LocalAnchorA);
                p2 = jd.BodyB.GetWorldPoint(jd.LocalAnchorB);
                d = p2 - p1;
                jd.Length = d.Length();
                _joints[7] = World.CreateJoint(jd);
            }
        }

        /// <inheritdoc />
        protected override void PreUpdate()
        {
            if (Input.GetKeyDown(KeyCode.B))
            {
                for (var i = 0; i < 4; ++i)
                {
                    if (_bodies[i] != null)
                    {
                        World.DestroyBody(_bodies[i]);
                        _bodies[i] = null;
                        break;
                    }
                }
            }

            if (Input.GetKeyDown(KeyCode.J))
            {
                for (var i = 0; i < 8; ++i)
                {
                    if (_joints[i] != null)
                    {
                        World.DestroyJoint(_joints[i]);
                        _joints[i] = null;
                        break;
                    }
                }
            }
        }

        /// <inheritdoc />
        protected override void PostStep()
        {
            DrawString("This demonstrates a soft distance joint.");
            DrawString("Press: (b) to delete a body, (j) to delete a joint");
        }
    }
}