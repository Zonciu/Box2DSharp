using System;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Testbed.Abstractions;
using Color = Box2DSharp.Common.Color;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.TestCases
{
    [TestCase("Joints", "Motor Joint")]
    public class MotorJoint : TestBase
    {
        private bool _go;

        private float _time;

        private Box2DSharp.Dynamics.Joints.MotorJoint _joint;

        public MotorJoint()
        {
            Body ground;
            {
                var bd = new BodyDef();
                ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(-20.0f, 0.0f), new Vector2(20.0f, 0.0f));

                var fd = new FixtureDef();
                fd.Shape = shape;

                ground.CreateFixture(fd);
            }

            // Define motorized body
            {
                var bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(0.0f, 8.0f);
                var body = World.CreateBody(bd);

                var shape = new PolygonShape();
                shape.SetAsBox(2.0f, 0.5f);

                var fd = new FixtureDef();
                fd.Shape = shape;
                fd.Friction = 0.6f;
                fd.Density = 2.0f;
                body.CreateFixture(fd);

                var mjd = new MotorJointDef();
                mjd.Initialize(ground, body);
                mjd.MaxForce = 1000.0f;
                mjd.MaxTorque = 1000.0f;
                _joint = (Box2DSharp.Dynamics.Joints.MotorJoint)World.CreateJoint(mjd);
            }

            _go = false;
            _time = 0.0f;
        }

        /// <inheritdoc />
        public override void OnKeyDown(KeyInputEventArgs keyInput)
        {
            if (keyInput.Key == KeyCodes.S)
            {
                _go = !_go;
            }
        }

        protected override void PreStep()
        {
            if (_go && 1 / TestSettings.Hertz > 0.0f)
            {
                _time += 1 / TestSettings.Hertz;
            }

            _linearOffset = new Vector2
            {
                X = 6.0f * (float)Math.Sin(2.0f * _time), Y = 8.0f + 4.0f * (float)Math.Sin(1.0f * _time)
            };

            var angularOffset = 4.0f * _time;

            _joint.SetLinearOffset(_linearOffset);
            _joint.SetAngularOffset(angularOffset);
        }

        private Vector2 _linearOffset;

        protected override void OnRender()
        {
            DrawString("Keys: (s) pause");

            Drawer.DrawPoint(_linearOffset, 4.0f, Color.FromArgb(230, 230, 230));
        }
    }
}