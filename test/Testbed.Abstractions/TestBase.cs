using System;
using System.Text.RegularExpressions;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Contacts;
using Box2DSharp.Dynamics.Joints;
using Joint = Box2DSharp.Dynamics.Joints.Joint;
using Random = System.Random;
using Vector2 = System.Numerics.Vector2;
using Color = Box2DSharp.Common.Color;

namespace Testbed.Abstractions
{
    public abstract class TestBase : IContactListener, IDestructionListener, IDisposable
    {
        public TestSettings TestSettings;

        public const int RandomLimit = 32767;

        public readonly Random Random = new Random();

        public readonly ContactPoint[] Points = new ContactPoint[2048];

        public int PointsCount;

        public Body GroundBody;

        public Profile MaxProfile;

        public MouseJoint MouseJoint;

        public Vector2 MouseWorld;

        public Profile TotalProfile;

        public World World;

        public int StepCount;

        public IDebugDrawer Drawer;

        public IInput Input;

        protected TestBase()
        {
            Regex.Replace(GetType().Name, @"(\B[A-Z])", " $1");
            World = new World(new Vector2(0, -10));
            World.SetContactListener(this);
            World.DestructionListener = this;
            GroundBody = World.CreateBody(new BodyDef());
            DumpLogger.Instance = new InternalDumpLogger();
        }

        #region Step

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

        protected virtual void PreStep()
        { }

        protected virtual void PostStep()
        { }

        public void Step()
        {
            if (TestSettings.Pause)
            {
                if (TestSettings.SingleStep)
                {
                    TestSettings.SingleStep = false;
                    WorldStep();
                }
            }
            else
            {
                WorldStep();
            }

            void WorldStep()
            {
                World.AllowSleep = TestSettings.EnableSleep;
                World.WarmStarting = TestSettings.EnableWarmStarting;
                World.ContinuousPhysics = TestSettings.EnableContinuous;
                World.SubStepping = TestSettings.EnableSubStepping;

                PointsCount = 0;

                PreStep();
                World.Step(1 / TestSettings.Hertz, TestSettings.VelocityIterations, TestSettings.PositionIterations);
                PostStep();
                ++StepCount;
            }
        }

        #endregion

        public virtual void OnKeyDown(KeyInputEventArgs keyInput)
        { }

        public virtual void OnKeyUp(KeyInputEventArgs keyInput)
        { }

        #region Render

        /// <summary>
        /// draw string and draw shape in here
        /// </summary>
        protected virtual void OnRender()
        { }

        private int _textLine = 30;

        private int _textIncrement = 13;

        public void DrawString(string text)
        {
            //_stringBuilder.AppendLine(text);
            Drawer.DrawString(5, _textLine, text);
            _textLine += _textIncrement;
        }

        public void DrawTitle(string title)
        {
            Drawer.DrawString(5, 5, title);
            _textLine = 26;
        }

        public void Render()
        {
            if (TestSettings.Pause)
            {
                DrawString("****PAUSED****");
            }

            // Statistics
            if (TestSettings.DrawStats)
            {
                DrawString($"bodies/contacts/joints = {World.BodyCount}/{World.ContactCount}/{World.JointCount}");
                DrawString($"proxies:{World.ProxyCount}/ height:{World.TreeHeight}/ balance: {World.TreeBalance} /quality: {World.TreeQuality}");
            }

            // Profile
            if (TestSettings.DrawProfile)
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
                if (!TestSettings.Pause && StepCount > 0)
                {
                    var scale = 1.0f / StepCount;
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
                DrawString($"collide [ave] (max) = {p.Collide} [{aveProfile.Collide}] ({MaxProfile.Collide})");
                DrawString($"solve [ave] (max) = {p.Solve} [{aveProfile.Solve}] ({MaxProfile.Solve})");
                DrawString($"solve init [ave] (max) = {p.SolveInit} [{aveProfile.SolveInit}] ({MaxProfile.SolveInit})");
                DrawString($"solve velocity [ave] (max) = {p.SolveVelocity} [{aveProfile.SolveVelocity}] ({MaxProfile.SolveVelocity})");
                DrawString($"solve position [ave] (max) = {p.SolvePosition} [{aveProfile.SolvePosition}] ({MaxProfile.SolvePosition})");
                DrawString($"solveTOI [ave] (max) = {p.SolveTOI} [{aveProfile.SolveTOI}] ({MaxProfile.SolveTOI})");
                DrawString($"broad-phase [ave] (max) = {p.Broadphase} [{aveProfile.Broadphase}] ({MaxProfile.Broadphase})");
            }

            OnRender();
            DrawWorld();
        }

        private void DrawWorld()
        {
            DrawFlag flags = 0;
            if (TestSettings.DrawShapes)
            {
                flags |= DrawFlag.DrawShape;
            }

            if (TestSettings.DrawJoints)
            {
                flags |= DrawFlag.DrawJoint;
            }

            if (TestSettings.DrawAABBs)
            {
                flags |= DrawFlag.DrawAABB;
            }

            if (TestSettings.DrawCOMs)
            {
                flags |= DrawFlag.DrawCenterOfMass;
            }

            if (TestSettings.DrawContactPoints)
            {
                flags |= DrawFlag.DrawContactPoint;
            }

            Drawer.Flags = flags;
            World.DebugDraw();
            if (TestSettings.DrawContactPoints)
            {
                const float ImpulseScale = 0.1f;
                const float AxisScale = 0.3f;
                for (var i = 0; i < PointsCount; ++i)
                {
                    var point = Points[i];
                    if (point.State == PointState.AddState)
                    {
                        // Add
                        Drawer.DrawPoint(point.Position, 10f, Color.FromArgb(77, 242, 77));
                    }
                    else if (point.State == PointState.PersistState)
                    {
                        // Persist
                        Drawer.DrawPoint(point.Position, 5f, Color.FromArgb(77, 77, 242));
                    }

                    if (TestSettings.DrawContactNormals)
                    {
                        var p1 = point.Position;
                        var p2 = p1 + AxisScale * point.Normal;
                        Drawer.DrawSegment(p1, p2, Color.FromArgb(230, 230, 230));
                    }
                    else if (TestSettings.DrawContactImpulse)
                    {
                        var p1 = point.Position;
                        var p2 = p1 + ImpulseScale * point.NormalImpulse * point.Normal;
                        Global.DebugDrawer.DrawSegment(p1, p2, Color.FromArgb(230, 230, 77));
                    }

                    if (TestSettings.DrawFrictionImpulse)
                    {
                        var tangent = MathUtils.Cross(point.Normal, 1.0f);
                        var p1 = point.Position;
                        var p2 = p1 + ImpulseScale * point.TangentImpulse * tangent;
                        Drawer.DrawSegment(p1, p2, Color.FromArgb(230, 230, 77));
                    }
                }
            }

            if (BombSpawning)
            {
                Drawer.DrawPoint(BombSpawnPoint, 4.0f, Color.Blue);
                Drawer.DrawSegment(MouseWorld, BombSpawnPoint, Color.FromArgb(203, 203, 203));
            }
        }

        #endregion

        #region Mouse Action

        private class MouseQueryCallback : IQueryCallback
        {
            public Fixture QueryFixture;

            public Vector2 Point;

            public void Reset(in Vector2 point)
            {
                QueryFixture = null;
                Point = point;
            }

            /// <inheritdoc />
            public bool QueryCallback(Fixture fixture1)
            {
                var body = fixture1.Body;
                if (body.BodyType == BodyType.DynamicBody)
                {
                    var inside = fixture1.TestPoint(Point);
                    if (inside)
                    {
                        QueryFixture = fixture1;

                        // We are done, terminate the query.
                        return false;
                    }
                }

                // Continue the query.
                return true;
            }
        }

        private readonly MouseQueryCallback _callback = new MouseQueryCallback();

        public void MouseDown(Vector2 p)
        {
            if (MouseJoint != null)
            {
                return;
            }

            MouseWorld = p;

            // Make a small box.
            var aabb = new AABB();
            var d = new Vector2(0.001f, 0.001f);
            aabb.LowerBound = p - d;
            aabb.UpperBound = p + d;

            // Query the world for overlapping shapes.
            _callback.Reset(p);
            World.QueryAABB(_callback, aabb);
            if (_callback.QueryFixture != null)
            {
                float frequencyHz = 5.0f;
                float dampingRatio = 0.7f;

                var body = _callback.QueryFixture.Body;
                var jd = new MouseJointDef
                {
                    BodyA = GroundBody,
                    BodyB = body,
                    Target = p,
                    MaxForce = 1000.0f * body.Mass
                };
                JointUtils.LinearStiffness(out jd.Stiffness, out jd.Damping, frequencyHz, dampingRatio, jd.BodyA, jd.BodyB);
                MouseJoint = (MouseJoint)World.CreateJoint(jd);
                body.IsAwake = true;
            }
        }

        public void MouseUp(Vector2 p)
        {
            MouseWorld = p;
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

        public void MouseMove(Vector2 p)
        {
            MouseWorld = p;
            MouseJoint?.SetTarget(p);
        }

        public void ShiftMouseDown(Vector2 p)
        {
            MouseWorld = p;
            if (MouseJoint != null)
            {
                return;
            }

            SpawnBomb(MouseWorld);
        }

        #endregion

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

        public void ShiftOrigin(Vector2 origin)
        {
            World.ShiftOrigin(origin);
        }

        public virtual void JointDestroyed(Joint joint)
        { }

        /// Random number in range [-1,1]
        public float RandomFloat()
        {
            float r = Random.Next() & RandomLimit;
            r /= RandomLimit;
            r = 2.0f * r - 1.0f;
            return r;
        }

        /// Random floating point number in range [lo, hi]
        public float RandomFloat(float lo, float hi)
        {
            float r = Random.Next() & RandomLimit;
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

            const float Multiplier = 30.0f;
            var vel = BombSpawnPoint - p;
            vel *= Multiplier;
            LaunchBomb(BombSpawnPoint, vel);
            BombSpawning = false;
        }

        public void LaunchBomb()
        {
            var p = new Vector2(Random.Next(-15, 15), 30.0f);
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

        public virtual void Dispose()
        {
            World?.Dispose();
            World = null;
        }
    }
}