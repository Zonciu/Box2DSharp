using System;
using System.Diagnostics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Testbed.Abstractions;
using Color = Box2DSharp.Common.Color;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.TestCases
{
    public class RayCastClosestCallback : IRayCastCallback
    {
        public bool Hit;

        public Vector2 Normal;

        public Vector2 Point;

        public RayCastClosestCallback()
        {
            Hit = false;
        }

        public float RayCastCallback(Fixture fixture, in Vector2 point, in Vector2 normal, float fraction)
        {
            var body = fixture.Body;
            var userData = (int?)body.UserData;
            if (userData == 1)
            {
                // By returning -1, we instruct the calling code to ignore this fixture and
                // continue the ray-cast to the next fixture.
                return -1.0f;
            }

            Hit = true;
            Point = point;
            Normal = normal;

            // By returning the current fraction, we instruct the calling code to clip the ray and
            // continue the ray-cast to the next fixture. WARNING: do not assume that fixtures
            // are reported in order. However, by clipping, we can always get the closest fixture.
            return fraction;
        }
    }

    public class RayCastAnyCallback : IRayCastCallback
    {
        public bool Hit;

        public Vector2 Normal;

        public Vector2 Point;

        public RayCastAnyCallback()
        {
            Hit = false;
        }

        public float RayCastCallback(Fixture fixture, in Vector2 point, in Vector2 normal, float _)
        {
            var body = fixture.Body;
            var userData = (int?)body.UserData;
            if (userData == 1)
            {
                // By returning -1, we instruct the calling code to ignore this fixture
                // and continue the ray-cast to the next fixture.
                return -1.0f;
            }

            Hit = true;
            Point = point;
            Normal = normal;

            // At this point we have a hit, so we know the ray is obstructed.
            // By returning 0, we instruct the calling code to terminate the ray-cast.
            return 0.0f;
        }
    }

    // This ray cast collects multiple hits along the ray. Polygon 0 is filtered.
    // The fixtures are not necessary reported in order, so we might not capture
    // the closest fixture.
    public class RayCastMultipleCallback : IRayCastCallback
    {
        public const int MaxCount = 3;

        public readonly Vector2[] Normals = new Vector2[MaxCount];

        public readonly Vector2[] Points = new Vector2[MaxCount];

        public int Count;

        public RayCastMultipleCallback()
        {
            Count = 0;
        }

        public float RayCastCallback(Fixture fixture, in Vector2 point, in Vector2 normal, float _)
        {
            var body = fixture.Body;
            var userData = (int?)body.UserData;
            if (userData == 1)
            {
                // By returning -1, we instruct the calling code to ignore this fixture
                // and continue the ray-cast to the next fixture.
                return -1.0f;
            }

            Debug.Assert(Count < MaxCount);

            Points[Count] = point;
            Normals[Count] = normal;
            ++Count;

            if (Count == MaxCount)
            {
                // At this point the buffer is full.
                // By returning 0, we instruct the calling code to terminate the ray-cast.
                return 0.0f;
            }

            // By returning 1, we instruct the caller to continue without clipping the ray.
            return 1.0f;
        }
    }

    [TestCase("Collision", "Ray Cast")]
    public class RayCast : TestBase
    {
        private const int MaxBodies = 256;

        private readonly Body[] _bodies = new Body[MaxBodies];

        private readonly CircleShape _circle = new CircleShape();

        private readonly EdgeShape _edge = new EdgeShape();

        private readonly PolygonShape[] _polygons = new PolygonShape[4]
        {
            new PolygonShape(),
            new PolygonShape(),
            new PolygonShape(),
            new PolygonShape()
        };

        protected float _degrees;

        private int _bodyIndex;

        protected Mode _mode;

        public RayCast()
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

            {
                _edge.SetTwoSided(new Vector2(-1.0f, 0.0f), new Vector2(1.0f, 0.0f));
            }

            _bodyIndex = 0;

            _degrees = 0.0f;

            _mode = Mode.Closest;
        }

        /// <inheritdoc />
        protected override void OnRender()
        {
            DrawString("Shape 1 is intentionally ignored by the ray");
            switch (_mode)
            {
            case Mode.Closest:
                DrawString("Ray-cast mode: closest - find closest fixture along the ray");
                break;

            case Mode.Any:
                DrawString("Ray-cast mode: any - check for obstruction");
                break;

            case Mode.Multiple:
                DrawString("Ray-cast mode: multiple - gather multiple fixtures");
                break;
            }

            var angle = Settings.Pi * _degrees / 180.0f;
            var L = 11.0f;
            var point1 = new Vector2(0.0f, 10.0f);

            var d = new Vector2(L * (float)Math.Cos(angle), L * (float)Math.Sin(angle));
            var point2 = point1 + d;

            switch (_mode)
            {
            case Mode.Closest:
            {
                var callback = new RayCastClosestCallback();
                World.RayCast(callback, point1, point2);

                if (callback.Hit)
                {
                    Drawer.DrawPoint(callback.Point, 5.0f, Color.FromArgb(102, 230, 102));
                    Drawer.DrawSegment(point1, callback.Point, Color.FromArgb(204, 204, 204));
                    var head = callback.Point + 0.5f * callback.Normal;
                    Drawer.DrawSegment(callback.Point, head, Color.FromArgb(230, 230, 102));
                }
                else
                {
                    Drawer.DrawSegment(point1, point2, Color.FromArgb(102, 204, 204));
                }

                break;
            }

            case Mode.Any:
            {
                var callback = new RayCastAnyCallback();
                World.RayCast(callback, point1, point2);

                if (callback.Hit)
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

                break;
            }

            case Mode.Multiple:
            {
                var callback = new RayCastMultipleCallback();
                World.RayCast(callback, point1, point2);
                Drawer.DrawSegment(point1, point2, Color.FromArgb(204, 204, 204));

                for (var i = 0; i < callback.Count; ++i)
                {
                    var p = callback.Points[i];
                    var n = callback.Normals[i];
                    Drawer.DrawPoint(p, 5.0f, Color.FromArgb(102, 230, 102));
                    Drawer.DrawSegment(point1, p, Color.FromArgb(204, 204, 204));
                    var head = p + 0.5f * n;
                    Drawer.DrawSegment(p, head, Color.FromArgb(230, 230, 102));
                }

                break;
            }
            }
        }

        protected void Create(int index)
        {
            if (_bodies[_bodyIndex] != null)
            {
                World.DestroyBody(_bodies[_bodyIndex]);
                _bodies[_bodyIndex] = null;
            }

            var x = RandomFloat(-10.0f, 10.0f);
            var y = RandomFloat(0.0f, 20.0f);
            var bd = new BodyDef();
            bd.Position.Set(x, y);
            bd.Angle = RandomFloat(-Settings.Pi, Settings.Pi);

            if (index == 4)
            {
                bd.AngularDamping = 0.02f;
            }

            _bodies[_bodyIndex] = World.CreateBody(bd);

            if (index < 4)
            {
                var fd = new FixtureDef {Shape = _polygons[index], Friction = 0.3f};
                fd.UserData = index + 1;
                _bodies[_bodyIndex].CreateFixture(fd);
            }
            else if (index < 5)
            {
                var fd = new FixtureDef {Shape = _circle, Friction = 0.3f};
                fd.UserData = index + 1;
                _bodies[_bodyIndex].CreateFixture(fd);
            }
            else
            {
                var fd = new FixtureDef {Shape = _edge, Friction = 0.3f};
                fd.UserData = index + 1;
                _bodies[_bodyIndex].CreateFixture(fd);
            }

            _bodyIndex = (_bodyIndex + 1) % MaxBodies;
        }

        protected void DestroyBody()
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

        protected enum Mode
        {
            Any = 0,

            Closest = 1,

            Multiple = 2
        }
    }
}