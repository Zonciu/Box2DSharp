using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using UnityEngine;
using Vector2 = System.Numerics.Vector2;

namespace Box2DSharp.Tests
{
    public class ShapeEditing : TestBase
    {
        private Body _body;

        private Fixture _fixture1;

        private Fixture _fixture2;

        private bool _sensor;

        private void Start()
        {
            {
                var bd = new BodyDef();
                var ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.Set(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);
            }

            {
                var bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(0.0f, 10.0f);
                _body = World.CreateBody(bd);

                var shape = new PolygonShape();
                shape.SetAsBox(4.0f, 4.0f, new Vector2(0.0f, 0.0f), 0.0f);
                _fixture1 = _body.CreateFixture(shape, 10.0f);
            }
            _fixture2 = null;
            _sensor = false;
        }

        /// <inheritdoc />
        protected override void OnUpdate()
        {
            DrawString("Press: (c) create a shape, (d) destroy a shape. (s) set sensor");
            DrawString($"sensor = {_sensor}");
            if (Input.GetKeyDown(KeyCode.C))
            {
                if (_fixture2 == null)
                {
                    var shape = new CircleShape();
                    shape.Radius = 3.0f;
                    shape.Position.Set(0.5f, -4.0f);
                    _fixture2 = _body.CreateFixture(shape, 10.0f);
                    _body.IsAwake = true;
                }
            }

            if (Input.GetKeyDown(KeyCode.D))
            {
                if (_fixture2 != null)
                {
                    _body.DestroyFixture(_fixture2);
                    _fixture2 = null;
                    _body.IsAwake = true;
                }
            }

            if (Input.GetKeyDown(KeyCode.S))
            {
                if (_fixture2 != null)
                {
                    _sensor = !_sensor;
                    _fixture2.IsSensor = _sensor;
                }
            }
        }
    }
}