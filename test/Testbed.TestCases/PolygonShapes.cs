using System;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Testbed.Abstractions;
using Color = Box2DSharp.Common.Color;
using Transform = Box2DSharp.Common.Transform;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.TestCases
{
    internal class PolyShapesCallback : IQueryCallback
    {
        private const int MaxCount = 4;

        private readonly IDrawer _drawer;

        public CircleShape Circle = new CircleShape();

        public int Count;

        public Transform Transform = new Transform();

        public PolyShapesCallback(IDrawer drawer)
        {
            _drawer = drawer;
            Count = 0;
        }

        /// Called for each fixture found in the query AABB.
        /// @return false to terminate the query.
        public bool QueryCallback(Fixture fixture)
        {
            if (Count == MaxCount)
            {
                return false;
            }

            var body = fixture.Body;
            var shape = fixture.Shape;

            var overlap = CollisionUtils.TestOverlap(
                shape,
                0,
                Circle,
                0,
                body.GetTransform(),
                Transform,
                fixture.Body.World.GJkProfile);

            if (overlap)
            {
                var color = Box2DSharp.Common.Color.FromArgb(0.95f, 0.95f, 0.6f);
                var center = body.GetWorldCenter();
                _drawer.DrawPoint(center, 5.0f, color);
                ++Count;
            }

            return true;
        }
    }

    [TestCase("Geometry", "Polygon Shapes")]
    public class PolygonShapes : TestBase
    {
        private const int MaxBodies = 256;

        private readonly Body[] _bodies = new Body[MaxBodies];

        private readonly PolygonShape[] _polygons = new PolygonShape[4]
        {
            new PolygonShape(),
            new PolygonShape(),
            new PolygonShape(),
            new PolygonShape()
        };

        private int _bodyIndex;

        private readonly CircleShape _circle = new CircleShape();

        public PolygonShapes()
        {
            // Ground body
            {
                var bd = new BodyDef();
                var ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);
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
        }

        private void Create(int index)
        {
            if (_bodies[_bodyIndex] != null)
            {
                World.DestroyBody(_bodies[_bodyIndex]);
                _bodies[_bodyIndex] = null;
            }

            var bd = new BodyDef();
            bd.BodyType = BodyType.DynamicBody;

            var x = RandomFloat(-2.0f, 2.0f);
            bd.Position.Set(x, 10.0f);
            bd.Angle = RandomFloat(-Settings.Pi, Settings.Pi);

            if (index == 4)
            {
                bd.AngularDamping = 0.02f;
            }

            _bodies[_bodyIndex] = World.CreateBody(bd);

            if (index < 4)
            {
                var fd = new FixtureDef();
                fd.Shape = _polygons[index];
                fd.Density = 1.0f;
                fd.Friction = 0.3f;
                _bodies[_bodyIndex].CreateFixture(fd);
            }
            else
            {
                var fd = new FixtureDef();
                fd.Shape = _circle;
                fd.Density = 1.0f;
                fd.Friction = 0.3f;

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

        /// <inheritdoc />
        public override void OnKeyDown(KeyInputEventArgs keyInput)
        {
            var k = -1;
            if (keyInput.Key == KeyCodes.D1)
            {
                k = 0;
            }

            if (keyInput.Key == KeyCodes.D2)
            {
                k = 1;
            }

            if (keyInput.Key == KeyCodes.D3)
            {
                k = 2;
            }

            if (keyInput.Key == KeyCodes.D4)
            {
                k = 3;
            }

            if (keyInput.Key == KeyCodes.D5)
            {
                k = 4;
            }

            if (k > -1)
            {
                Create(k);
            }

            if (keyInput.Key == KeyCodes.A)
            {
                for (var i = 0; i < MaxBodies; i += 2)
                {
                    if (_bodies[i] != null)
                    {
                        var isEnabled = _bodies[i].IsEnabled;
                        _bodies[i].IsEnabled = !isEnabled;
                    }
                }
            }

            if (keyInput.Key == KeyCodes.D)
            {
                DestroyBody();
            }
        }

        /// <inheritdoc />
        protected override void OnRender()
        {
            DrawString("Press 1-5 to drop stuff");
            DrawString("Press 'a' to (de)activate some bodies");
            DrawString("Press 'd' to destroy a body");

            var callback = new PolyShapesCallback(Drawer) {Circle = {Radius = 2.0f}};
            callback.Circle.Position.Set(0.0f, 1.1f);
            callback.Circle.ComputeAABB(out var aabb, callback.Transform, 0);
            callback.Transform.SetIdentity();

            World.QueryAABB(callback, aabb);

            var color = Color.FromArgb(102, 178, 204);
            Drawer.DrawCircle(callback.Circle.Position, callback.Circle.Radius, color);
        }
    }
}