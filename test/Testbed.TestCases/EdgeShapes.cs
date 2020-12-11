using System;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Testbed.Abstractions;
using Color = Box2DSharp.Common.Color;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.TestCases
{
    internal class EdgeShapesCallback : IRayCastCallback
    {
        public Fixture Fixture;

        public Vector2 Normal;

        public Vector2 Point;

        /// <inheritdoc />
        public float RayCastCallback(Fixture fixture, in Vector2 point, in Vector2 normal, float fraction)
        {
            Fixture = fixture;
            Point = point;
            Normal = normal;

            return fraction;
        }
    }

    [TestCase("Geometry", "Edge Shapes")]
    public class EdgeShapes : TestBase
    {
        private const int MaxBodies = 256;

        private readonly Body[] _bodies = new Body[MaxBodies];

        private readonly PolygonShape[] _polygons = new PolygonShape[4].Fill();

        private float _angle;

        private int _bodyIndex;

        private CircleShape _circle = new CircleShape();

        public EdgeShapes()
        {
            // Ground body
            {
                var bd = new BodyDef();
                var ground = World.CreateBody(bd);

                var x1 = -20.0f;
                var y1 = 2.0f * (float)Math.Cos(x1 / 10.0f * Settings.Pi);
                for (var i = 0; i < 80; ++i)
                {
                    var x2 = x1 + 0.5f;
                    var y2 = 2.0f * (float)Math.Cos(x2 / 10.0f * Settings.Pi);

                    var shape = new EdgeShape();
                    shape.SetTwoSided(new Vector2(x1, y1), new Vector2(x2, y2));
                    ground.CreateFixture(shape, 0.0f);

                    x1 = x2;
                    y1 = y2;
                }
            }

            {
                var vertices = new Vector2[3];
                vertices[0].Set(-0.5f, 0.0f);
                vertices[1].Set(0.5f, 0.0f);
                vertices[2].Set(0.0f, 1.5f);
                _polygons[0].Set(vertices);
            }

            {
                var vertices = new Vector2[3];
                vertices[0].Set(-0.1f, 0.0f);
                vertices[1].Set(0.1f, 0.0f);
                vertices[2].Set(0.0f, 1.5f);
                _polygons[1].Set(vertices);
            }

            {
                var w = 1.0f;
                var b = w / (2.0f + (float)Math.Sqrt(2.0f));
                var s = (float)Math.Sqrt(2.0f) * b;

                var vertices = new Vector2[8];
                vertices[0].Set(0.5f * s, 0.0f);
                vertices[1].Set(0.5f * w, b);
                vertices[2].Set(0.5f * w, b + s);
                vertices[3].Set(0.5f * s, w);
                vertices[4].Set(-0.5f * s, w);
                vertices[5].Set(-0.5f * w, b + s);
                vertices[6].Set(-0.5f * w, b);
                vertices[7].Set(-0.5f * s, 0.0f);

                _polygons[2].Set(vertices);
            }

            {
                _polygons[3].SetAsBox(0.5f, 0.5f);
            }

            {
                _circle.Radius = 0.5f;
            }

            _bodyIndex = 0;
            _angle = 0.0f;
        }

        private void Create(int index)
        {
            if (_bodies[_bodyIndex] != null)
            {
                World.DestroyBody(_bodies[_bodyIndex]);
                _bodies[_bodyIndex] = null;
            }

            var bd = new BodyDef();

            var x = RandomFloat(-10.0f, 10.0f);
            var y = RandomFloat(10.0f, 20.0f);
            bd.Position.Set(x, y);
            bd.Angle = RandomFloat(-Settings.Pi, Settings.Pi);
            bd.BodyType = BodyType.DynamicBody;

            if (index == 4)
            {
                bd.AngularDamping = 0.02f;
            }

            _bodies[_bodyIndex] = World.CreateBody(bd);

            if (index < 4)
            {
                var fd = new FixtureDef();
                fd.Shape = _polygons[index];
                fd.Friction = 0.3f;
                fd.Density = 20.0f;
                _bodies[_bodyIndex].CreateFixture(fd);
            }
            else
            {
                var fd = new FixtureDef();
                fd.Shape = _circle;
                fd.Friction = 0.3f;
                fd.Density = 20.0f;
                _bodies[_bodyIndex].CreateFixture(fd);
            }

            _bodyIndex = (_bodyIndex + 1) % MaxBodies;
        }

        private void DestroyBody()
        {
            for (var i = 0; i < MaxBodies; ++i)
            {
                if (_bodies[i] != null)
                {
                    World.DestroyBody(_bodies[i]);
                    _bodies[i] = null;
                    return;
                }
            }
        }

        protected override void OnRender()
        {
            DrawString("Press 1-5 to drop stuff");

            var advanceRay = TestSettings.Pause == false || TestSettings.SingleStep;

            var L = 25.0f;
            var point1 = new Vector2(0.0f, 10.0f);

            var d = new Vector2(L * (float)Math.Cos(_angle), -L * (float)Math.Abs(Math.Sin(_angle)));
            var point2 = point1 + d;

            var callback = new EdgeShapesCallback();

            World.RayCast(callback, point1, point2);

            if (callback.Fixture != null)
            {
                Drawer.DrawPoint(callback.Point, 5.0f, Color.FromArgb(102, 230, 102));
                Drawer.DrawSegment(point1, callback.Point, Color.FromArgb(204, 204, 204));
                var head = callback.Point + 0.5f * callback.Normal;
                Drawer.DrawSegment(callback.Point, head, Color.FromArgb(230, 230, 102));
            }
            else
            {
                Drawer.DrawSegment(point1, point2, Color.FromArgb(204, 204, 204));
            }

            if (advanceRay)
            {
                _angle += 0.25f * Settings.Pi / 180.0f;
            }
        }

        /// <inheritdoc />
        public override void OnKeyDown(KeyInputEventArgs keyInput)
        {
            var k = -1;
            if (keyInput.Key == KeyCodes.D1)
            {
                k = 0;
            }
            else if (keyInput.Key == KeyCodes.D2)
            {
                k = 1;
            }
            else if (keyInput.Key == KeyCodes.D3)
            {
                k = 2;
            }
            else if (keyInput.Key == KeyCodes.D4)
            {
                k = 3;
            }
            else if (keyInput.Key == KeyCodes.D5)
            {
                k = 4;
            }
            else if (keyInput.Key == KeyCodes.D6)
            {
                k = 5;
            }

            if (k != -1)
            {
                Create(k);
            }

            if (keyInput.Key == KeyCodes.D)
            {
                DestroyBody();
            }
        }
    }
}