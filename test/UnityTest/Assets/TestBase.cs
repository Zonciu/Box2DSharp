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
using Joint = Box2DSharp.Dynamics.Joints.Joint;
using Random = System.Random;
using Vector2 = System.Numerics.Vector2;
using Color = System.Drawing.Color;

namespace Box2DSharp
{
    public abstract class TestBase : MonoBehaviour, IContactListener, IDestructionListener
    {
        public const int RandomLimit = 32767;

        private readonly Random _random = new Random();

        public readonly ContactPoint[] Points = new ContactPoint[2048];

        private float _deltaTime;

        public int PointsCount;

        public Vector3 Diference;

        public bool Drag;

        public FrameManager FrameManager;

        public Body GroundBody;

        public Camera MainCamera;

        public Profile MaxProfile;

        public MouseJoint MouseJoint;

        public Vector2 MouseWorld;

        public UnityEngine.Vector2 MouseWorlcPosition;

        public Vector3 Origin;

        public TestSettings TestSettings;

        public Profile TotalProfile;

        public World World;

        public BoxDrawer Drawer { get; private set; }

        public virtual void BeginContact(Contact contact)
        { }

        public virtual void EndContact(Contact contact)
        { }

        public virtual void PreSolve(Contact contact, in Manifold oldManifold)
        {
            ref readonly var manifold = ref contact.Manifold;

            if (manifold.PointCount == 0)
            {
                return;
            }

            var fixtureA = contact.FixtureA;
            var fixtureB = contact.FixtureB;

            var state1 = new PointState [Settings.MaxManifoldPoints];
            var state2 = new PointState [Settings.MaxManifoldPoints];
            CollisionUtils.GetPointStates(state1, state2, oldManifold, manifold);

            contact.GetWorldManifold(out var worldManifold);

            for (var i = 0; i < manifold.PointCount && PointsCount < Points.Length; ++i)
            {
                var cp = new ContactPoint
                {
                    FixtureA = fixtureA, FixtureB = fixtureB,
                    Position = worldManifold.Points[i], Normal = worldManifold.Normal,
                    State = state2[i], NormalImpulse = manifold.Points[i].NormalImpulse,
                    TangentImpulse = manifold.Points[i].TangentImpulse, Separation = worldManifold.Separations[i]
                };
                Points[PointsCount] = cp;
                ++PointsCount;
            }
        }

        /// <inheritdoc />
        public virtual void PostSolve(Contact contact, in ContactImpulse impulse)
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
                Job = Tick,
                Interval = 1 / TestSettings.Frequency
            };
            World.SetContactListener(this);
            World.DestructionListener = this;
            World.Drawer = TestSettings.WorldDrawer;
            Drawer = TestSettings.WorldDrawer;
            DumpLogger.Instance = new UnityLogger();

            // DrawString
            _rect = new Rect(0, 0, Screen.width, Screen.height * 2f / 100f);
            _style = new GUIStyle
            {
                alignment = TextAnchor.UpperLeft, fontSize = Screen.height * 2 / 100,
                normal = {textColor = new UnityEngine.Color(0.0f, 0.0f, 0.5f, 1.0f)}
            };
        }

        private void Start()
        {
            Create();
        }

        protected virtual void Create()
        { }

        protected virtual void PreStep()
        { }

        protected virtual void PostStep()
        { }

        protected virtual void PreUpdate()
        { }

        protected virtual void PostUpdate()
        { }

        protected virtual void PreLateUpdate()
        { }

        protected virtual void PostLateUpdate()
        { }

        private void Tick()
        {
            PreStep();
            PointsCount = 0;
            World.AllowSleep = TestSettings.Sleep;
            World.WarmStarting = TestSettings.WarmStarting;
            World.ContinuousPhysics = TestSettings.TimeOfImpact;
            World.SubStepping = TestSettings.SubStepping;
            var timeStep = 1f / TestSettings.Frequency;
            if (TestSettings.Pause)
            {
                if (TestSettings.SingleStep)
                {
                    TestSettings.SingleStep = false;
                }
                else
                {
                    timeStep = 0.0f;
                }

                DrawString("****PAUSED****");
            }

            World.Step(timeStep, TestSettings.VelocityIteration, TestSettings.PositionIteration);
            PostStep();
        }

        protected void Update()
        {
            PreUpdate();

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

            FrameManager.Tick();

            // Mouse left drag
            MouseWorld = MainCamera.ScreenToWorldPoint(Input.mousePosition).ToVector2();
            MouseJoint?.SetTarget(MouseWorld);

            if (TestSettings.EnableMouseAction)
            {
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
                    Diference = MainCamera.ScreenToWorldPoint(Input.mousePosition) - MainCamera.transform.position;
                    if (Drag == false)
                    {
                        Drag = true;
                        Origin = MainCamera.ScreenToWorldPoint(Input.mousePosition);
                    }
                }
                else
                {
                    Drag = false;
                }

                if (Drag)
                {
                    MainCamera.transform.position = Origin - Diference;
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
            }

            _deltaTime += (Time.unscaledDeltaTime - _deltaTime) * 0.1f;
            PostUpdate();
        }

        private void LateUpdate()
        {
            PreLateUpdate();
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
                MaxProfile.Step = Math.Max(MaxProfile.Step, p.Step);
                MaxProfile.Collide = Math.Max(MaxProfile.Collide, p.Collide);
                MaxProfile.Solve = Math.Max(MaxProfile.Solve, p.Solve);
                MaxProfile.SolveInit = Math.Max(MaxProfile.SolveInit, p.SolveInit);
                MaxProfile.SolveVelocity = Math.Max(MaxProfile.SolveVelocity, p.SolveVelocity);
                MaxProfile.SolvePosition = Math.Max(MaxProfile.SolvePosition, p.SolvePosition);
                MaxProfile.SolveTOI = Math.Max(MaxProfile.SolveTOI, p.SolveTOI);
                MaxProfile.Broadphase = Math.Max(MaxProfile.Broadphase, p.Broadphase);

                TotalProfile.Step += p.Step;
                TotalProfile.Collide += p.Collide;
                TotalProfile.Solve += p.Solve;
                TotalProfile.SolveInit += p.SolveInit;
                TotalProfile.SolveVelocity += p.SolveVelocity;
                TotalProfile.SolvePosition += p.SolvePosition;
                TotalProfile.SolveTOI += p.SolveTOI;
                TotalProfile.Broadphase += p.Broadphase;

                var aveProfile = new Profile();
                if (FrameManager.FrameCount > 0)
                {
                    var scale = 1.0f / FrameManager.FrameCount;
                    aveProfile.Step = scale * TotalProfile.Step;
                    aveProfile.Collide = scale * TotalProfile.Collide;
                    aveProfile.Solve = scale * TotalProfile.Solve;
                    aveProfile.SolveInit = scale * TotalProfile.SolveInit;
                    aveProfile.SolveVelocity = scale * TotalProfile.SolveVelocity;
                    aveProfile.SolvePosition = scale * TotalProfile.SolvePosition;
                    aveProfile.SolveTOI = scale * TotalProfile.SolveTOI;
                    aveProfile.Broadphase = scale * TotalProfile.Broadphase;
                }

                DrawString($"step [ave] (max) = {p.Step} [{aveProfile.Step}] ({MaxProfile.Step})");
                DrawString(
                    $"collide [ave] (max) = {p.Collide} [{aveProfile.Collide}] ({MaxProfile.Collide})");
                DrawString($"solve [ave] (max) = {p.Solve} [{aveProfile.Solve}] ({MaxProfile.Solve})");
                DrawString(
                    $"solve init [ave] (max) = {p.SolveInit} [{aveProfile.SolveInit}] ({MaxProfile.SolveInit})");
                DrawString(
                    $"solve velocity [ave] (max) = {p.SolveVelocity} [{aveProfile.SolveVelocity}] ({MaxProfile.SolveVelocity})");
                DrawString(
                    $"solve position [ave] (max) = {p.SolvePosition} [{aveProfile.SolvePosition}] ({MaxProfile.SolvePosition})");
                DrawString(
                    $"solveTOI [ave] (max) = {p.SolveTOI} [{aveProfile.SolveTOI}] ({MaxProfile.SolveTOI})");
                DrawString(
                    $"broad-phase [ave] (max) = {p.Broadphase} [{aveProfile.Broadphase}] ({MaxProfile.Broadphase})");
            }

            if (_stringBuilder.Length > 0)
            {
                _text = _stringBuilder.ToString();
                _stringBuilder.Clear();
            }

            PostLateUpdate();
        }

        private void DrawWorld()
        {
            World.DrawDebugData();
            if (TestSettings.ContactPoint)
            {
                const float impulseScale = 0.1f;
                const float axisScale = 0.3f;
                var debugDraw = TestSettings.WorldDrawer;
                for (var i = 0; i < PointsCount; ++i)
                {
                    var point = Points[i];
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
                        debugDraw.DrawSegment(p1, p2, System.Drawing.Color.FromArgb(230, 230, 77));
                    }
                }
            }

            if (BombSpawning)
            {
                Drawer.DrawPoint(BombSpawnPoint, 4.0f, System.Drawing.Color.Blue);
                Drawer.DrawSegment(MouseWorld, BombSpawnPoint, System.Drawing.Color.FromArgb(203, 203, 203));
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
                        var inside = fixture1.TestPoint(p);
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

        public void ShiftMouseDown()
        {
            if (MouseJoint != null)
            {
                return;
            }

            SpawnBomb(MouseWorld);
        }

        private void OnDestroy()
        {
            FrameManager.Dispose();
            World = null;
        }

        public virtual void JointDestroyed(Joint joint)
        { }

        /// Random number in range [-1,1]
        public float RandomFloat()
        {
            float r = _random.Next() & RandomLimit;
            r /= RandomLimit;
            r = 2.0f * r - 1.0f;
            return r;
        }

        /// Random floating point number in range [lo, hi]
        public float RandomFloat(float lo, float hi)
        {
            float r = _random.Next() & RandomLimit;
            r /= RandomLimit;
            r = (hi - lo) * r + lo;
            return r;
        }

        public struct ContactPoint
        {
            public Fixture FixtureA;

            public Fixture FixtureB;

            public Vector2 Normal;

            public Vector2 Position;

            public PointState State;

            public float NormalImpulse;

            public float TangentImpulse;

            public float Separation;
        }

        #region Bomb

        protected Vector2 BombSpawnPoint;

        protected bool BombSpawning;

        protected Body Bomb;

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

        public void LaunchBomb()
        {
            var p = new Vector2(UnityEngine.Random.Range(-15.0f, 15.0f), 30.0f);
            var v = -5.0f * p;
            LaunchBomb(p, v);
        }

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

        #endregion

        #region Drawing

        private Rect _rect;

        private GUIStyle _style;

        /// <summary>
        ///     Display Physic Frame
        /// </summary>
        protected virtual void OnGUI()
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

        public void DrawAABB(AABB aabb, Color color)
        {
            var vs = new Vector2 [4];
            vs[0].Set(aabb.LowerBound.X, aabb.LowerBound.Y);
            vs[1].Set(aabb.UpperBound.X, aabb.LowerBound.Y);
            vs[2].Set(aabb.UpperBound.X, aabb.UpperBound.Y);
            vs[3].Set(aabb.LowerBound.X, aabb.UpperBound.Y);

            Drawer.DrawPolygon(vs, 4, color);
        }

        #endregion
    }
}