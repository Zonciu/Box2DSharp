using System;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Testbed.Abstractions;
using Transform = Box2DSharp.Common.Transform;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.TestCases
{
    /// <summary>
    /// This test shows how to apply forces and torques to a body.
    /// It also shows how to use the friction joint that can be useful
    /// for overhead games.
    /// </summary>
    [TestCase("Forces", "ApplyForce")]
    public class ApplyForce : TestBase
    {
        private Body _body;

        public ApplyForce()
        {
            World.Gravity = new Vector2(0.0f, 0.0f);

            const float restitution = 0.4f;

            Body ground;
            {
                var bd = new BodyDef();
                bd.Position.Set(0.0f, 20.0f);
                ground = World.CreateBody(bd);

                var shape = new EdgeShape();

                var sd = new FixtureDef();
                sd.Shape = shape;
                sd.Density = 0.0f;
                sd.Restitution = restitution;

                // Left vertical
                shape.SetTwoSided(new Vector2(-20.0f, -20.0f), new Vector2(-20.0f, 20.0f));
                ground.CreateFixture(sd);

                // Right vertical
                shape.SetTwoSided(new Vector2(20.0f, -20.0f), new Vector2(20.0f, 20.0f));
                ground.CreateFixture(sd);

                // Top horizontal
                shape.SetTwoSided(new Vector2(-20.0f, 20.0f), new Vector2(20.0f, 20.0f));
                ground.CreateFixture(sd);

                // Bottom horizontal
                shape.SetTwoSided(new Vector2(-20.0f, -20.0f), new Vector2(20.0f, -20.0f));
                ground.CreateFixture(sd);
            }

            {
                var xf1 = new Transform();
                xf1.Rotation.Set(0.3524f * Settings.Pi);
                xf1.Position = xf1.Rotation.GetXAxis();

                var vertices = new Vector2[3];
                vertices[0] = MathUtils.Mul(xf1, new Vector2(-1.0f, 0.0f));
                vertices[1] = MathUtils.Mul(xf1, new Vector2(1.0f, 0.0f));
                vertices[2] = MathUtils.Mul(xf1, new Vector2(0.0f, 0.5f));

                var poly1 = new PolygonShape();
                poly1.Set(vertices);

                var sd1 = new FixtureDef();
                sd1.Shape = poly1;
                sd1.Density = 2.0f;

                var xf2 = new Transform();
                xf2.Rotation.Set(-0.3524f * Settings.Pi);
                xf2.Position = -xf2.Rotation.GetXAxis();

                vertices[0] = MathUtils.Mul(xf2, new Vector2(-1.0f, 0.0f));
                vertices[1] = MathUtils.Mul(xf2, new Vector2(1.0f, 0.0f));
                vertices[2] = MathUtils.Mul(xf2, new Vector2(0.0f, 0.5f));

                var poly2 = new PolygonShape();
                poly2.Set(vertices);

                var sd2 = new FixtureDef();
                sd2.Shape = poly2;
                sd2.Density = 2.0f;

                var bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position = new Vector2(0.0f, 3.0f);
                bd.Angle = Settings.Pi;
                bd.AllowSleep = false;
                _body = World.CreateBody(bd);
                _body.CreateFixture(sd1);
                _body.CreateFixture(sd2);

                var gravity = 10.0f;
                var I = _body.Inertia;
                var mass = _body.Mass;

                // Compute an effective radius that can be used to
                // set the max torque for a friction joint
                // For a circle: I = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
                var radius = (float)Math.Sqrt(2.0f * I / mass);

                FrictionJointDef jd = new FrictionJointDef();
                jd.BodyA = ground;
                jd.BodyB = _body;
                jd.LocalAnchorA.SetZero();
                jd.LocalAnchorB = _body.GetLocalCenter();
                jd.CollideConnected = true;
                jd.MaxForce = 0.5f * mass * gravity;
                jd.MaxTorque = 0.2f * mass * radius * gravity;

                World.CreateJoint(jd);
            }

            {
                var shape = new PolygonShape();
                shape.SetAsBox(0.5f, 0.5f);

                var fd = new FixtureDef();
                fd.Shape = shape;
                fd.Density = 1.0f;
                fd.Friction = 0.3f;

                for (var i = 0; i < 10; ++i)
                {
                    var bd = new BodyDef();
                    bd.BodyType = BodyType.DynamicBody;

                    bd.Position.Set(0.0f, 7.0f + 1.54f * i);
                    var body = World.CreateBody(bd);

                    body.CreateFixture(fd);

                    var gravity = 10.0f;
                    var I = body.Inertia;
                    var mass = body.Mass;

                    // For a circle: I = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
                    var radius = (float)Math.Sqrt(2.0f * I / mass);

                    var jd = new FrictionJointDef();
                    jd.LocalAnchorA.SetZero();
                    jd.LocalAnchorB.SetZero();
                    jd.BodyA = ground;
                    jd.BodyB = body;
                    jd.CollideConnected = true;
                    jd.MaxForce = mass * gravity;
                    jd.MaxTorque = 0.1f * mass * radius * gravity;

                    World.CreateJoint(jd);
                }
            }
        }

        /// <inheritdoc />
        protected override void PreStep()
        {
            if (Input.IsKeyDown(KeyCodes.W))
            {
                var f = _body.GetWorldVector(new Vector2(0.0f, -50.0f));
                var p = _body.GetWorldPoint(new Vector2(0.0f, 3.0f));
                _body.ApplyForce(f, p, true);
            }

            if (Input.IsKeyDown(KeyCodes.A))
            {
                _body.ApplyTorque(10.0f, true);
            }

            if (Input.IsKeyDown(KeyCodes.D))
            {
                _body.ApplyTorque(-10.0f, true);
            }
        }

        protected override void OnRender()
        {
            DrawString("Forward (W), Turn (A) and (D)");
        }
    }
}