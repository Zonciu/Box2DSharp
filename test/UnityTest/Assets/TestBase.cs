using System;
using System.Text;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Collision.Shapes;
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
using Debug = System.Diagnostics.Debug;
using Joint = Box2DSharp.Dynamics.Joints.Joint;
using MathUtils = Box2DSharp.Common.MathUtils;
using Transform = UnityEngine.Transform;

namespace Box2DSharp
{
    public abstract class TestBase : MonoBehaviour, IContactListener, IDestructionListener
    {
        public TestSettings TestSettings;

        public World World;

        public Body GroundBody;

        public Camera MainCamera;

        public FrameManager FrameManager;

        public IDrawer Drawer { get; private set; }

        private float _deltaTime;

        private readonly Profile _maxProfile = new Profile();

        private readonly Profile _totalProfile = new Profile();

        public Vector2 MouseWorld;

        public MouseJoint MouseJoint;

        private Vector3 _origin;

        private Vector3 _diference;

        private bool _drag = false;

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
            World.DestructionListener = this;
            World.Drawer = TestSettings.WorldDrawer;
            Drawer = TestSettings.WorldDrawer;
            Logger.SetLogger(new UnityLogger());

            // DrawString
            _rect = new Rect(0, 0, Screen.width, Screen.height * 2f / 100f);
            _style = new GUIStyle
            {
                alignment = TextAnchor.UpperLeft, fontSize = Screen.height * 2 / 100,
                normal = {textColor = new UnityEngine.Color(0.0f, 0.0f, 0.5f, 1.0f)}
            };
        }

        protected virtual void PreStep()
        { }

        protected virtual void PostStep()
        { }

        protected virtual void OnUpdate()
        { }

        protected virtual void OnLateUpdate()
        { }

        protected void Update()
        {
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

            // Launch Bomb
            if (Input.GetKeyDown(KeyCode.Space))
            {
                LaunchBomb();
            }

            OnUpdate();
            FrameManager.Tick();

            // Mouse left drag
            MouseWorld = MainCamera.ScreenToWorldPoint(Input.mousePosition).ToVector2();
            MouseJoint?.SetTarget(MouseWorld);
            if (Input.GetMouseButtonDown((int) MouseButton.LeftMouse))
            {
                if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
                {
                    ShiftMouseDown();
                }
                else
                {
                    MouseDown();
                }
            }

            if (Input.GetMouseButtonUp((int) MouseButton.LeftMouse))
            {
                MouseUp();
            }

            // Mouse right move camera
            if (Input.GetMouseButton((int) MouseButton.RightMouse))
            {
                _diference = (MainCamera.ScreenToWorldPoint(Input.mousePosition)) - MainCamera.transform.position;
                if (_drag == false)
                {
                    _drag = true;
                    _origin = MainCamera.ScreenToWorldPoint(Input.mousePosition);
                }
            }
            else
            {
                _drag = false;
            }

            if (_drag)
            {
                MainCamera.transform.position = _origin - _diference;
            }

            // Mouse wheel zoom
            //Zoom out
            if (Input.GetAxis("Mouse ScrollWheel") < 0)
            {
                if (MainCamera.orthographicSize > 1)
                {
                    MainCamera.orthographicSize += 1f;
                }
                else
                {
                    MainCamera.orthographicSize += 0.1f;
                }
            }

            //Zoom in
            if (Input.GetAxis("Mouse ScrollWheel") > 0)
            {
                if (MainCamera.orthographicSize > 1)
                {
                    MainCamera.orthographicSize -= 1f;
                }
                else if (MainCamera.orthographicSize > 0.2f)
                {
                    MainCamera.orthographicSize -= 0.1f;
                }
            }

            _deltaTime += (Time.unscaledDeltaTime - _deltaTime) * 0.1f;
        }

        private void LateUpdate()
        {
            OnLateUpdate();
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

            // Statistics
            if (TestSettings.Statistics)
            {
                DrawString(
                    $"bodies/contacts/joints = {World.BodyCount}/{World.ContactCount}/{World.JointCount}");
                DrawString(
                    $"proxies:{World.ProxyCount}/ height:{World.TreeHeight}/ balance: {World.TreeBalance} /quality: {World.TreeQuality}");
            }

            // Profile
            if (TestSettings.Profile)
            {
                var p = World.Profile;

                // Track maximum profile times
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

            if (_stringBuilder.Length > 0)
            {
                _text = _stringBuilder.ToString();
                _stringBuilder.Clear();
            }
        }

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
                        debugDraw.DrawPoint(point.Position, 10f, Color.FromArgb(77, 242, 77));
                    }
                    else if (point.State == PointState.PersistState)
                    {
                        // Persist
                        debugDraw.DrawPoint(point.Position, 5f, Color.FromArgb(77, 77, 242));
                    }

                    if (TestSettings.ContactNormals)
                    {
                        var p1 = point.Position;
                        var p2 = p1 + axisScale * point.Normal;
                        debugDraw.DrawSegment(p1, p2, Color.FromArgb(230, 230, 230));
                    }
                    else if (TestSettings.ContactImpulse)
                    {
                        var p1 = point.Position;
                        var p2 = p1 + impulseScale * point.NormalImpulse * point.Normal;
                        debugDraw.DrawSegment(p1, p2, Color.FromArgb(230, 230, 77));
                    }

                    if (TestSettings.FrictionImpulse)
                    {
                        var tangent = MathUtils.Cross(point.Normal, 1.0f);
                        var p1 = point.Position;
                        var p2 = p1 + impulseScale * point.TangentImpulse * tangent;
                        debugDraw.DrawSegment(p1, p2, Color.FromArgb(230, 230, 77));
                    }
                }
            }

            if (BombSpawning)
            {
                Drawer.DrawPoint(BombSpawnPoint, 4.0f, Color.Blue);
                Drawer.DrawSegment(MouseWorld, BombSpawnPoint, Color.FromArgb(203, 203, 203));
            }
        }

        public void MouseDown()
        {
            if (MouseJoint != null)
            {
                return;
            }

            var p = MouseWorld;

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
                    var body = fixture1.Body;
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
                var body = fixture.Body;
                var md = new MouseJointDef
                {
                    BodyA = GroundBody, BodyB = body,
                    Target = p, MaxForce = 1000.0f * body.Mass
                };
                MouseJoint = (MouseJoint) World.CreateJoint(md);
                body.IsAwake = true;
            }
        }

        public void MouseUp()
        {
            if (MouseJoint != null)
            {
                World.DestroyJoint(MouseJoint);
                MouseJoint = null;
            }

            if (BombSpawning)
            {
                CompleteBombSpawn(MouseWorld);
            }
        }

        public void MouseMove()
        {
            MouseJoint?.SetTarget(MouseWorld);
        }

        protected Vector2 BombSpawnPoint;

        protected bool BombSpawning;

        public void SpawnBomb(Vector2 worldPt)
        {
            BombSpawnPoint = worldPt;
            BombSpawning = true;
        }

        public void CompleteBombSpawn(Vector2 p)
        {
            if (BombSpawning == false)
            {
                return;
            }

            const float multiplier = 30.0f;
            var vel = BombSpawnPoint - p;
            vel *= multiplier;
            LaunchBomb(BombSpawnPoint, vel);
            BombSpawning = false;
        }

        public void ShiftMouseDown()
        {
            if (MouseJoint != null)
            {
                return;
            }

            SpawnBomb(MouseWorld);
        }

        public void LaunchBomb()
        {
            var p = new Vector2(UnityEngine.Random.Range(-15.0f, 15.0f), 30.0f);
            var v = -5.0f * p;
            LaunchBomb(p, v);
        }

        protected Body Bomb;

        public void LaunchBomb(Vector2 position, Vector2 velocity)
        {
            if (Bomb != default)
            {
                World.DestroyBody(Bomb);
                Bomb = default;
            }

            var bd = new BodyDef
            {
                BodyType = BodyType.DynamicBody, Position = position,
                Bullet = true
            };
            Bomb = World.CreateBody(bd);
            Bomb.SetLinearVelocity(velocity);

            var circle = new CircleShape {Radius = 0.3f};

            var fd = new FixtureDef
            {
                Shape = circle, Density = 20.0f,
                Restitution = 0.0f
            };

            Bomb.CreateFixture(fd);
        }

        private Rect _rect;

        private GUIStyle _style;

        /// <summary>
        /// Display Physic Frame
        /// </summary>
        private void OnGUI()
        {
            DrawString();
        }

        private readonly StringBuilder _stringBuilder = new StringBuilder();

        private string _text;

        private void DrawString()
        {
            GUI.Label(_rect, _text, _style);
        }

        public void DrawString(string text)
        {
            _stringBuilder.AppendLine(text);
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
                    FixtureA = fixtureA, FixtureB = fixtureB,
                    Position = worldManifold.Points[i], Normal = worldManifold.Normal,
                    State = state2[i], NormalImpulse = manifold.Points[i].NormalImpulse,
                    TangentImpulse = manifold.Points[i].TangentImpulse, Separation = worldManifold.Separations[i]
                };
                _points[_pointsCount] = cp;
                ++_pointsCount;
            }
        }

        /// <inheritdoc />
        public void PostSolve(Contact contact, in ContactImpulse impulse)
        { }

        private void OnDestroy()
        {
            FrameManager.Dispose();
            World = null;
        }

        public virtual void JointDestroyed(Joint joint)
        { }

        /// <inheritdoc />
        public void SayGoodbye(Joint joint)
        {
            if (MouseJoint == joint)
            {
                MouseJoint = null;
            }
            else
            {
                JointDestroyed(joint);
            }
        }

        /// <inheritdoc />
        public void SayGoodbye(Fixture fixture)
        {
            /* Do nothing */
        }

        private struct ContactPoint
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