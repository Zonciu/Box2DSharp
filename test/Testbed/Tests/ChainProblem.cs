using System.Linq;
using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Testbed.Basics;

namespace Testbed.Tests
{
    [TestCase("Bugs", "Chain Problem")]
    public class ChainProblem : Test
    {
        public ChainProblem()
        {
            //dump
            {
                Vector2 g = new Vector2(0.000000000000000e+00f, -1.000000000000000e+01f);
                World.Gravity = g;
                Body[] bodies = new Body[2];
                Joint[] joints = new Joint[0];
                {
                    BodyDef bd = new BodyDef
                    {
                        BodyType = (BodyType)0,
                        Position = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f),
                        Angle = 0.000000000000000e+00f,
                        LinearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f),
                        AngularVelocity = 0.000000000000000e+00f,
                        LinearDamping = 0.000000000000000e+00f,
                        AngularDamping = 0.000000000000000e+00f,
                        AllowSleep = true,
                        Awake = true,
                        FixedRotation = false,
                        Bullet = false,
                        Enabled = true,
                        GravityScale = 1.000000000000000e+00f
                    };
                    bodies[0] = World.CreateBody(bd);

                    {
                        FixtureDef fd = new FixtureDef();
                        fd.Friction = 2.000000029802322e-01f;
                        fd.Restitution = 0.000000000000000e+00f;
                        fd.Density = 0.000000000000000e+00f;
                        fd.IsSensor = false;
                        fd.Filter.CategoryBits = 1;
                        fd.Filter.MaskBits = 65535;
                        fd.Filter.GroupIndex = 0;
                        ChainShape shape = new ChainShape();
                        Vector2[] vs =
                        {
                            new Vector2(0.000000000000000e+00f, 1.000000000000000e+00f),
                            new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f),
                            new Vector2(4.000000000000000e+00f, 0.000000000000000e+00f)
                        };

                        shape.CreateChain(vs);
                        shape.PrevVertex.Set(4.719737010713663e-34f, 8.266340761211261e-34f);
                        shape.NextVertex.Set(1.401298464324817e-45f, 8.266340761211261e-34f);
                        shape.HasPrevVertex = false;
                        shape.HasNextVertex = false;

                        fd.Shape = shape;

                        bodies[0].CreateFixture(fd);
                    }
                }
                {
                    BodyDef bd = new BodyDef();
                    bd.BodyType = (BodyType)2;
                    bd.Position.Set(6.033980250358582e-01f, 3.028350114822388e+00f);
                    bd.Angle = 0.000000000000000e+00f;
                    bd.LinearVelocity.Set(0.000000000000000e+00f, 0.000000000000000e+00f);
                    bd.AngularVelocity = 0.000000000000000e+00f;
                    bd.LinearDamping = 0.000000000000000e+00f;
                    bd.AngularDamping = 0.000000000000000e+00f;
                    bd.AllowSleep = true;
                    bd.Awake = true;
                    bd.FixedRotation = false;
                    bd.Bullet = true;
                    bd.Enabled = true;
                    bd.GravityScale = 1.000000000000000e+00f;
                    bodies[1] = World.CreateBody(bd);

                    {
                        FixtureDef fd = new FixtureDef();
                        fd.Friction = 2.000000029802322e-01f;
                        fd.Restitution = 0.000000000000000e+00f;
                        fd.Density = 1.000000000000000e+01f;
                        fd.IsSensor = false;
                        fd.Filter.CategoryBits = 1;
                        fd.Filter.MaskBits = 65535;
                        fd.Filter.GroupIndex = 0;
                        PolygonShape shape = new PolygonShape();
                        Vector2[] vs = new Vector2[8];
                        vs[0].Set(5.000000000000000e-01f, -3.000000000000000e+00f);
                        vs[1].Set(5.000000000000000e-01f, 3.000000000000000e+00f);
                        vs[2].Set(-5.000000000000000e-01f, 3.000000000000000e+00f);
                        vs[3].Set(-5.000000000000000e-01f, -3.000000000000000e+00f);
                        shape.Set(vs, 4);

                        fd.Shape = shape;

                        bodies[1].CreateFixture(fd);
                    }
                }
                joints = null;
                bodies = null;
            }
        }
    }
}