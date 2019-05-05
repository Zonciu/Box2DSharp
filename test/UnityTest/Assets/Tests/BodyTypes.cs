using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using UnityEngine;
using Vector2 = System.Numerics.Vector2;

namespace Box2DSharp.Tests
{
    public class BodyTypes : Test
    {
        private Body _attachment;

        private Body _platform;

        private float _speed;

        public BodyTypes()
        {
            Body ground;
            {
                var bd = new BodyDef();
                ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.Set(new Vector2(-20.0f, 0.0f), new Vector2(20.0f, 0.0f));

                var fd = new FixtureDef();
                fd.Shape = shape;

                ground.CreateFixture(fd);
            }

            // Define attachment
            {
                var bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(0.0f, 3.0f);
                _attachment = World.CreateBody(bd);

                var shape = new PolygonShape();
                shape.SetAsBox(0.5f, 2.0f);
                _attachment.CreateFixture(shape, 2.0f);
            }

            // Define platform
            {
                var bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(-4.0f, 5.0f);
                _platform = World.CreateBody(bd);

                var shape = new PolygonShape();
                shape.SetAsBox(0.5f, 4.0f, new Vector2(4.0f, 0.0f), 0.5f * Settings.Pi);

                var fd = new FixtureDef();
                fd.Shape = shape;
                fd.Friction = 0.6f;
                fd.Density = 2.0f;
                _platform.CreateFixture(fd);

                var rjd = new RevoluteJointDef();
                rjd.Initialize(_attachment, _platform, new Vector2(0.0f, 5.0f));
                rjd.MaxMotorTorque = 50.0f;
                rjd.EnableMotor = true;
                World.CreateJoint(rjd);

                var pjd = new PrismaticJointDef();
                pjd.Initialize(ground, _platform, new Vector2(0.0f, 5.0f), new Vector2(1.0f, 0.0f));

                pjd.MaxMotorForce = 1000.0f;
                pjd.EnableMotor = true;
                pjd.LowerTranslation = -10.0f;
                pjd.UpperTranslation = 10.0f;
                pjd.EnableLimit = true;

                World.CreateJoint(pjd);

                _speed = 3.0f;
            }

            // Create a payload
            {
                var bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(0.0f, 8.0f);
                var body = World.CreateBody(bd);

                var shape = new PolygonShape();
                shape.SetAsBox(0.75f, 0.75f);

                var fd = new FixtureDef();
                fd.Shape = shape;
                fd.Friction = 0.6f;
                fd.Density = 2.0f;

                body.CreateFixture(fd);
            }
        }

        public override void OnRender()
        {
            DrawString("Keys: (d) dynamic, (s) static, (k) kinematic");
        }

        protected override void OnStep()
        {
            // Drive the kinematic body.
            if (_platform.BodyType == BodyType.KinematicBody)
            {
                var p = _platform.GetTransform().Position;
                var v = _platform.LinearVelocity;

                if (p.X < -10.0f && v.X < 0.0f || p.X > 10.0f && v.X > 0.0f)
                {
                    v.X = -v.X;
                    _platform.SetLinearVelocity(v);
                }
            }

            if (Input.GetKeyDown(KeyCode.D))
            {
                _platform.BodyType = BodyType.DynamicBody;
            }

            if (Input.GetKeyDown(KeyCode.S))
            {
                _platform.BodyType = BodyType.StaticBody;
            }

            if (Input.GetKeyDown(KeyCode.K))
            {
                _platform.BodyType = BodyType.KinematicBody;
                _platform.SetLinearVelocity(new Vector2(-_speed, 0.0f));
                _platform.SetAngularVelocity(0.0f);
            }
        }
    }
}