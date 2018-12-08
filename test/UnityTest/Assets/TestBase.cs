using System;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Contacts;
using Box2DSharp.Dynamics.Joints;
using Box2DSharp.Inspection;
using UnityEngine;
using UnityEngine.Experimental.UIElements;
using Logger = Box2DSharp.Common.Logger;
using Vector2 = System.Numerics.Vector2;
using Color = System.Drawing.Color;
using MathUtils = Box2DSharp.Common.MathUtils;

namespace Box2DSharp
{
    public abstract class TestBase : MonoBehaviour, IContactListener
    {
        public TestSettings TestSettings;

        public World World;

        public Body GroundBody;

        public Camera MainCamera;

        public FrameManager FrameManager;

        private void Awake()
        {
            if (!Camera.main)
            {
                throw new NullReferenceException("Require Main Camera: Camera.main");
            }

            TestSettings = FindObjectOfType<TestSettings>();
            MainCamera = Camera.main;
            World = new World(new Vector2(0, -10));
            GroundBody = World.CreateBody(new BodyDef());
            FrameManager = new FrameManager
            {
                Job = () =>
                {
                    PreStep();
                    _pointsCount = 0;
                    World.AllowSleep = TestSettings.Sleep;
                    World.WarmStarting = TestSettings.WarmStarting;
                    World.ContinuousPhysics = TestSettings.TimeOfImpact;
                    World.SubStepping = TestSettings.SubStepping;
                    World.Step(
                        1f / TestSettings.Frequency,
                        TestSettings.VelocityIteration,
                        TestSettings.PositionIteration);
                    PostStep();
                },
                Interval = 1 / TestSettings.Frequency
            };
            World.SetContactListener(this);
            World.SetDebugDrawer(TestSettings.WorldDrawer);
            Logger.SetLogger(new UnityLogger());

            // DrawString
            _line = 0f;
            _lineHeight = Screen.height * 2 / 100f;
            _style = new GUIStyle
            {
                alignment = TextAnchor.UpperLeft,
                fontSize = Screen.height * 2 / 100,
                normal = {textColor = new UnityEngine.Color(0.0f, 0.0f, 0.5f, 1.0f)}
            };
        }

        protected virtual void PreStep()
        { }

        protected virtual void PostStep()
        { }

        private void Update()
        {
            FrameManager.Tick();

            // Mouse left drag
            Mouse = MainCamera.ScreenToWorldPoint(Input.mousePosition);
            _mouseJoint?.SetTarget(Mouse.ToVector2());
            if (Input.GetMouseButtonDown((int) MouseButton.LeftMouse))
            {
                MouseDown();
            }

            if (Input.GetMouseButtonUp((int) MouseButton.LeftMouse))
            {
                MouseUp();
            }

            // Mouse right move camera
            if (Input.GetMouseButton((int) MouseButton.RightMouse))
            {
                var h = Input.GetAxis("Mouse X");
                var v = Input.GetAxis("Mouse Y");
                MainCamera.transform.Translate(-h, -v, 0, Space.World);
            }

            // Mouse wheel zoom
            //Zoom out
            if (Input.GetAxis("Mouse ScrollWheel") < 0)
            {
                MainCamera.fieldOfView += 2;
                MainCamera.orthographicSize += 1F;
            }

            //Zoom in
            if (Input.GetAxis("Mouse ScrollWheel") > 0)
            {
                MainCamera.fieldOfView -= 2;
                MainCamera.orthographicSize -= 1F;
            }

            _deltaTime += (Time.unscaledDeltaTime - _deltaTime) * 0.1f;
        }

        private void LateUpdate()
        {
            DrawFlag flags = 0;
            if (TestSettings.Shape)
            {
                flags |= DrawFlag.DrawShape;
            }

            if (TestSettings.Joint)
            {
                flags |= DrawFlag.DrawJoint;
            }

            if (TestSettings.AABB)
            {
                flags |= DrawFlag.DrawAABB;
            }

            if (TestSettings.Pair)
            {
                flags |= DrawFlag.DrawPair;
            }

            if (TestSettings.CenterOfMass)
            {
                flags |= DrawFlag.DrawCenterOfMass;
            }

            if (TestSettings.ContactPoint)
            {
                flags |= DrawFlag.DrawContactPoint;
            }

            TestSettings.WorldDrawer.Flags = flags;
            DrawWorld();
        }

        private readonly Color _c1 = Color.FromArgb(77, 242, 77);

        private readonly Color _c2 = Color.FromArgb(77, 77, 242);

        private readonly Color _c3 = Color.FromArgb(230, 230, 230);

        private readonly Color _c4 = Color.FromArgb(230, 230, 77);

        private void DrawWorld()
        {
            World.DrawDebugData();
            if (TestSettings.ContactPoint)
            {
                const float impulseScale = 0.1f;
                const float axisScale = 0.3f;
                var debugDraw = TestSettings.WorldDrawer;
                for (var i = 0; i < _pointsCount; ++i)
                {
                    var point = _points[i];
                    if (point.State == PointState.AddState)
                    {
                        // Add
                        debugDraw.DrawPoint(point.Position, 10f, _c1);
                    }
                    else if (point.State == PointState.PersistState)
                    {
                        // Persist
                        debugDraw.DrawPoint(point.Position, 5f, _c2);
                    }

                    if (TestSettings.ContactNormals)
                    {
                        var p1 = point.Position;
                        var p2 = p1 + axisScale * point.Normal;
                        debugDraw.DrawSegment(p1, p2, _c3);
                    }
                    else if (TestSettings.ContactImpulse)
                    {
                        var p1 = point.Position;
                        var p2 = p1 + impulseScale * point.NormalImpulse * point.Normal;
                        debugDraw.DrawSegment(p1, p2, _c4);
                    }

                    if (TestSettings.FrictionImpulse)
                    {
                        var tangent = MathUtils.Cross(point.Normal, 1.0f);
                        var p1 = point.Position;
                        var p2 = p1 + impulseScale * point.TangentImpulse * tangent;
                        debugDraw.DrawSegment(p1, p2, _c4);
                    }
                }
            }
        }

        public Vector3 Mouse;

        private MouseJoint _mouseJoint;

        private void MouseDown()
        {
            if (_mouseJoint != null)
            {
                return;
            }

            var p = Mouse.ToVector2();

            // Make a small box.
            var aabb = new AABB();
            var d = new Vector2(0.001f, 0.001f);
            aabb.LowerBound = p - d;
            aabb.UpperBound = p + d;

            // Query the world for overlapping shapes.
            Fixture fixture = null;
            World.QueryAABB(
                fixture1 =>
                {
                    var body = fixture1.GetBody();
                    if (body.BodyType == BodyType.DynamicBody)
                    {
                        bool inside = fixture1.TestPoint(p);
                        if (inside)
                        {
                            fixture = fixture1;

                            // We are done, terminate the query.
                            return false;
                        }
                    }

                    // Continue the query.
                    return true;
                },
                aabb);

            if (fixture != null)
            {
                var body = fixture.GetBody();
                var md = new MouseJointDef
                {
                    BodyA = GroundBody,
                    BodyB = body,
                    Target = p,
                    MaxForce = 1000.0f * body.Mass
                };
                _mouseJoint = (MouseJoint) World.CreateJoint(md);
                body.IsAwake = true;
            }
        }

        private void MouseUp()
        {
            if (_mouseJoint != null)
            {
                World.DestroyJoint(_mouseJoint);
                _mouseJoint = null;
            }
        }

        private void OnDestroy()
        {
            FrameManager.Dispose();
            World = null;
        }

        private float _deltaTime;

        private Profile _maxProfile = new Profile();

        private Profile _totalProfile = new Profile();

        private Rect _rect;

        private float _line;

        private float _lineHeight;

        private GUIStyle _style;

        /// <summary>
        /// Display Physic Frame
        /// </summary>
        private void OnGUI()
        {
            _line = 0;
            var drawer = TestSettings.DebugDrawer;

            // FPS
            {
                var msec = _deltaTime * 1000.0f;
                var fps = 1.0f / _deltaTime;
                var text = $"{msec:0.0} ms ({fps:0.} fps)";
                DrawString(text);
            }

            // Frame
            {
                DrawString($"{FrameManager.FrameCount} Frames");
            }

            // Statistics
            if (TestSettings.Statistics)
            {
                DrawString(
                    $"bodies/contacts/joints = {World.BodyCount}/{World.ContactCount}/{World.JointCount}");
                DrawString(
                    $"proxies:{World.ProxyCount}/ height:{World.TreeHeight}/ balance: {World.TreeBalance} /quality: {World.TreeQuality}");
            }

            // Profile
            {
                // Track maximum profile times
                {
                    var p = World.Profile;
                    _maxProfile.Step = Math.Max(_maxProfile.Step, p.Step);
                    _maxProfile.Collide = Math.Max(_maxProfile.Collide, p.Collide);
                    _maxProfile.Solve = Math.Max(_maxProfile.Solve, p.Solve);
                    _maxProfile.SolveInit = Math.Max(_maxProfile.SolveInit, p.SolveInit);
                    _maxProfile.SolveVelocity = Math.Max(_maxProfile.SolveVelocity, p.SolveVelocity);
                    _maxProfile.SolvePosition = Math.Max(_maxProfile.SolvePosition, p.SolvePosition);
                    _maxProfile.SolveTOI = Math.Max(_maxProfile.SolveTOI, p.SolveTOI);
                    _maxProfile.Broadphase = Math.Max(_maxProfile.Broadphase, p.Broadphase);

                    _totalProfile.Step += p.Step;
                    _totalProfile.Collide += p.Collide;
                    _totalProfile.Solve += p.Solve;
                    _totalProfile.SolveInit += p.SolveInit;
                    _totalProfile.SolveVelocity += p.SolveVelocity;
                    _totalProfile.SolvePosition += p.SolvePosition;
                    _totalProfile.SolveTOI += p.SolveTOI;
                    _totalProfile.Broadphase += p.Broadphase;
                }

                if (TestSettings.Profile)
                {
                    var p = World.Profile;

                    var aveProfile = new Profile();
                    if (FrameManager.FrameCount > 0)
                    {
                        var scale = 1.0f / FrameManager.FrameCount;
                        aveProfile.Step = scale * _totalProfile.Step;
                        aveProfile.Collide = scale * _totalProfile.Collide;
                        aveProfile.Solve = scale * _totalProfile.Solve;
                        aveProfile.SolveInit = scale * _totalProfile.SolveInit;
                        aveProfile.SolveVelocity = scale * _totalProfile.SolveVelocity;
                        aveProfile.SolvePosition = scale * _totalProfile.SolvePosition;
                        aveProfile.SolveTOI = scale * _totalProfile.SolveTOI;
                        aveProfile.Broadphase = scale * _totalProfile.Broadphase;
                    }

                    DrawString($"step [ave] (max) = {p.Step} [{aveProfile.Step}] ({_maxProfile.Step})");
                    DrawString(
                        $"collide [ave] (max) = {p.Collide} [{aveProfile.Collide}] ({_maxProfile.Collide})");
                    DrawString($"solve [ave] (max) = {p.Solve} [{aveProfile.Solve}] ({_maxProfile.Solve})");
                    DrawString(
                        $"solve init [ave] (max) = {p.SolveInit} [{aveProfile.SolveInit}] ({_maxProfile.SolveInit})");
                    DrawString(
                        $"solve velocity [ave] (max) = {p.SolveVelocity} [{aveProfile.SolveVelocity}] ({_maxProfile.SolveVelocity})");
                    DrawString(
                        $"solve position [ave] (max) = {p.SolvePosition} [{aveProfile.SolvePosition}] ({_maxProfile.SolvePosition})");
                    DrawString(
                        $"solveTOI [ave] (max) = {p.SolveTOI} [{aveProfile.SolveTOI}] ({_maxProfile.SolveTOI})");
                    DrawString(
                        $"broad-phase [ave] (max) = {p.Broadphase} [{aveProfile.Broadphase}] ({_maxProfile.Broadphase})");
                }
            }
        }

        public void DrawString(string text)
        {
            var rect = new Rect(0, _line, Screen.width, Screen.height * 2f / 100f);
            GUI.Label(rect, text, _style);
            _line += _lineHeight;
        }

        private readonly ContactPoint[] _points = new ContactPoint[2048];

        private int _pointsCount = 0;

        /// <inheritdoc />
        public void BeginContact(Contact contact)
        { }

        /// <inheritdoc />
        public void EndContact(Contact contact)
        { }

        /// <inheritdoc />
        public void PreSolve(Contact contact, in Manifold oldManifold)
        {
            ref readonly var manifold = ref contact.GetManifold();

            if (manifold.PointCount == 0)
            {
                return;
            }

            var fixtureA = contact.GetFixtureA();
            var fixtureB = contact.GetFixtureB();

            var state1 = new PointState [Settings.MaxManifoldPoints];
            var state2 = new PointState [Settings.MaxManifoldPoints];
            CollisionUtils.GetPointStates(state1, state2, oldManifold, manifold);

            contact.GetWorldManifold(out var worldManifold);

            for (var i = 0; i < manifold.PointCount && _pointsCount < _points.Length; ++i)
            {
                var cp = new ContactPoint
                {
                    FixtureA = fixtureA,
                    FixtureB = fixtureB,
                    Position = worldManifold.Points[i],
                    Normal = worldManifold.Normal,
                    State = state2[i],
                    NormalImpulse = manifold.Points[i].NormalImpulse,
                    TangentImpulse = manifold.Points[i].TangentImpulse,
                    Separation = worldManifold.Separations[i]
                };
                _points[_pointsCount] = cp;
                ++_pointsCount;
            }
        }

        /// <inheritdoc />
        public void PostSolve(Contact contact, in ContactImpulse impulse)
        { }

        struct ContactPoint
        {
            public Fixture FixtureA;

            public Fixture FixtureB;

            public Vector2 Normal;

            public Vector2 Position;

            public PointState State;

            public float NormalImpulse;

            public float TangentImpulse;

            public float Separation;
        };
    }
}