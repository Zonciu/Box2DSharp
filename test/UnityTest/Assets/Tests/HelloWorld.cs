using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Inspection;
using UnityEngine;
using Random = System.Random;
using Vector2 = System.Numerics.Vector2;

namespace Box2DSharp.Tests
{
    public class HelloWorld : TestBase
    {
        public double X;

        public double Y;

        public double Angle;

        public bool IsAwake;

        private Body _body;

        private void Start()
        {
            var groundBodyDef = new BodyDef {BodyType = BodyType.StaticBody};
            groundBodyDef.Position.Set(0.0f, -10.0f);

            var groundBody = World.CreateBody(groundBodyDef);

            var groundBox = new PolygonShape();
            groundBox.SetAsBox(1000.0f, 10.0f);

            groundBody.CreateFixture(groundBox, 0.0f);

            // Define the dynamic body. We set its position and call the body factory.
            var bodyDef = new BodyDef {BodyType = BodyType.DynamicBody};

            bodyDef.Position.Set(0, 4f);

            var dynamicBox = new PolygonShape();
            dynamicBox.SetAsBox(1f, 1f, Vector2.Zero, 45f);

            // Define the dynamic body fixture.
            var fixtureDef = new FixtureDef
            {
                shape    = dynamicBox,
                density  = 1.0f,
                friction = 0.3f
            };

            // Set the box density to be non-zero, so it will be dynamic.

            // Override the default friction.

            // Add the shape to the body.
            var body = World.CreateBody(bodyDef);
            body.CreateFixture(fixtureDef);

            Debug.Log("World initialized");
            _body = body;

            for (int i = 0; i < 500; i++)
            {
                bodyDef.Position = new Vector2(UnityEngine.Random.Range(0, 500), UnityEngine.Random.Range(0, 500));
                bodyDef.Angle    = UnityEngine.Random.Range(0, 360);
                World.CreateBody(bodyDef).CreateFixture(fixtureDef);
            }
        }

        private void FixedUpdate()
        {
            var t = _body.GetTransform();
            X     = t.Position.X;
            Y     = t.Position.Y;
            Angle = t.Rotation.Angle;
            var v  = _body.LinearVelocity;
            var vd = _body.LinearDamping;
            var a  = _body.AngularVelocity;
            var ad = _body.AngularDamping;
            IsAwake = _body.IsAwake;

            //Debug.Log($"{v},{vd} ; {a},{ad} ; {IsAwake}");
        }
    }
}