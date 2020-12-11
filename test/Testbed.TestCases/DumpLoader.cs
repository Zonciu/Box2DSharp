using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Bugs", "Dump Loader")]
    public class DumpLoader : TestBase
    {
        public DumpLoader()
        {
            //Source code dump of Box2D scene: issue304-minimal-case.rube
            //
            //  Created by R.U.B.E 1.3.0
            //  Using Box2D version 2.3.0
            //  Wed April 3 2013 04:33:28
            //
            //  This code is originally intended for use in the Box2D testbed,
            //  but you can easily use it in other applications by providing
            //  a World for use as the 'World' variable in the code below.

            Vector2 g = new Vector2(0.000000000000000e+00f, -1.000000000000000e+01f);
            World.Gravity = g;
            Body[] bodies = new Body[3];
            Joint[] joints = new Joint[0];
            {
                BodyDef bd = new BodyDef();
                bd.BodyType = (BodyType)0;
                bd.Position.Set(2.587699890136719e-02f, 5.515012264251709e+00f);
                bd.Angle = 0.000000000000000e+00f;
                bd.LinearVelocity.Set(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.AngularVelocity = 0.000000000000000e+00f;
                bd.LinearDamping = 0.000000000000000e+00f;
                bd.AngularDamping = 0.000000000000000e+00f;
                bd.AllowSleep = true;
                bd.Awake = true;
                bd.FixedRotation = false;
                bd.Bullet = false;
                bd.Enabled = true;
                bd.GravityScale = 1.000000000000000e+00f;
                bodies[0] = World.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.Friction = 2.000000029802322e-01f;
                    fd.Restitution = 0.000000000000000e+00f;
                    fd.Density = 1.000000000000000e+00f;
                    fd.IsSensor = false;
                    fd.Filter.CategoryBits = (1);
                    fd.Filter.MaskBits = (65535);
                    fd.Filter.GroupIndex = (0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[8];
                    vs[0].Set(7.733039855957031e-01f, -1.497260034084320e-01f);
                    vs[1].Set(-4.487270116806030e-01f, 1.138330027461052e-01f);
                    vs[2].Set(-1.880589962005615e+00f, -1.365900039672852e-01f);
                    vs[3].Set(3.972740173339844e-01f, -3.897832870483398e+00f);
                    shape.Set(vs, 4);

                    fd.Shape = shape;

                    bodies[0].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.BodyType = (BodyType)(2);
                bd.Position.Set(-3.122138977050781e-02f, 7.535382270812988e+00f);
                bd.Angle = -1.313644275069237e-02f;
                bd.LinearVelocity.Set(8.230687379837036e-01f, 7.775862514972687e-02f);
                bd.AngularVelocity = 3.705333173274994e-02f;
                bd.LinearDamping = 0.000000000000000e+00f;
                bd.AngularDamping = 0.000000000000000e+00f;
                bd.AllowSleep = true;
                bd.Awake = true;
                bd.FixedRotation = false;
                bd.Bullet = false;
                bd.Enabled = true;
                bd.GravityScale = 1.000000000000000e+00f;
                bodies[1] = World.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.Friction = 5.000000000000000e-01f;
                    fd.Restitution = 0.000000000000000e+00f;
                    fd.Density = 5.000000000000000e+00f;
                    fd.IsSensor = false;
                    fd.Filter.CategoryBits = (1);
                    fd.Filter.MaskBits = (65535);
                    fd.Filter.GroupIndex = (0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[8];
                    vs[0].Set(3.473900079727173e+00f, -2.009889930486679e-01f);
                    vs[1].Set(3.457079887390137e+00f, 3.694039955735207e-02f);
                    vs[2].Set(-3.116359949111938e+00f, 2.348500071093440e-03f);
                    vs[3].Set(-3.109960079193115e+00f, -3.581250011920929e-01f);
                    vs[4].Set(-2.590820074081421e+00f, -5.472509860992432e-01f);
                    vs[5].Set(2.819370031356812e+00f, -5.402340292930603e-01f);
                    shape.Set(vs, 6);

                    fd.Shape = shape;

                    bodies[1].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.BodyType = (BodyType)(2);
                bd.Position.Set(-7.438077926635742e-01f, 6.626811981201172e+00f);
                bd.Angle = -1.884713363647461e+01f;
                bd.LinearVelocity.Set(1.785794943571091e-01f, 3.799796104431152e-07f);
                bd.AngularVelocity = -5.908820639888290e-06f;
                bd.LinearDamping = 0.000000000000000e+00f;
                bd.AngularDamping = 0.000000000000000e+00f;
                bd.AllowSleep = true;
                bd.Awake = true;
                bd.FixedRotation = false;
                bd.Bullet = false;
                bd.Enabled = true;
                bd.GravityScale = 1.000000000000000e+00f;
                bodies[2] = World.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.Friction = 9.499999880790710e-01f;
                    fd.Restitution = 0.000000000000000e+00f;
                    fd.Density = 1.000000000000000e+01f;
                    fd.IsSensor = false;
                    fd.Filter.CategoryBits = (1);
                    fd.Filter.MaskBits = (65535);
                    fd.Filter.GroupIndex = (-3);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[8];
                    vs[0].Set(1.639146506786346e-01f, 4.428443685173988e-02f);
                    vs[1].Set(-1.639146655797958e-01f, 4.428443685173988e-02f);
                    vs[2].Set(-1.639146655797958e-01f, -4.428443312644958e-02f);
                    vs[3].Set(1.639146357774734e-01f, -4.428444057703018e-02f);
                    shape.Set(vs, 4);

                    fd.Shape = shape;

                    bodies[2].CreateFixture(fd);
                }
            }

            joints = null;
            bodies = null;
        }
    }
}