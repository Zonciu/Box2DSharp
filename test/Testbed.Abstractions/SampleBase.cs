using System;
using Box2DSharp;

namespace Testbed.Abstractions
{
    public abstract class SampleBase : IDisposable
    {
        public readonly Settings Settings;

        public IDraw Draw;

        public IInput Input;

        public float TimeStep;

        public int TaskCount;

        public BodyId GroundBodyId;

        public WorldId WorldId;

        public JointId MouseJointId;

        public int StepCount;

        public Profile MaxProfile;

        public Profile TotalProfile;

        protected SampleBase(Settings settings)
        {
            Settings = settings;
            var worldDef = WorldDef.DefaultWorldDef();
            worldDef.WorkerCount = settings.WorkerCount;
            worldDef.EnqueueTask = EnqueueTaskCallback;
            worldDef.FinishTask = FinishTaskCallback;
            worldDef.UserTaskContext = this;
            worldDef.EnableSleep = settings.EnableSleep;
            WorldId = World.CreateWorld(worldDef);
            MouseJointId = JointId.NullId;
            GroundBodyId = BodyId.NullId;
            MaxProfile = new();
            TotalProfile = new();
            B2Random.Shared.Reset();
        }

        #region Step

        public virtual void OnInitialized()
        { }

        public virtual void PreStep()
        { }

        public virtual void PostStep()
        { }

        private static object? EnqueueTaskCallback(TaskCallback task, int itemCount, int minRange, object taskContext, object userContext)
        {
            task(0, itemCount, 0, taskContext);
            return null;
        }

        private static void FinishTaskCallback(object userTask, object userContext)
        { }

        public virtual void Step()
        {
            TimeStep = Settings.Hertz > 0.0f ? 1.0f / Settings.Hertz : 0f;

            if (Settings.Pause)
            {
                if (Settings.SingleStep)
                {
                    Settings.SingleStep = false;
                }
                else
                {
                    TimeStep = 0;
                }
            }

            World.SetEnableSleeping(WorldId, Settings.EnableSleep);
            World.SetEnableWarmStarting(WorldId, Settings.EnableWarmStarting);
            World.SetEnableContinuous(WorldId, Settings.EnableContinuous);

            World.Step(WorldId, TimeStep, Settings.SubStepCount);
            TaskCount = 0;
        }

        #endregion

        public virtual void UpdateUI()
        { }

        public void ResetProfile()
        {
            TotalProfile = new();
            MaxProfile = new();
            StepCount = 0;
        }

        #region Render

        /// <summary>
        /// draw string and draw shape in here
        /// </summary>
        protected virtual void OnRender()
        { }

        private int _textLine = 30;

        private const int TextIncrement = 18;

        public void DrawString(string text)
        {
            Draw.DrawString(5, _textLine, text);
            _textLine += TextIncrement;
        }

        public void DrawTitle(string title)
        {
            Draw.DrawString(5, 5, title);
            _textLine = 26;
        }

        public void Render()
        {
            if (Settings.Pause)
            {
                DrawString("****PAUSED****");
            }

            Draw.DebugDraw.DrawingBounds = Global.Camera.GetViewBounds();
            Draw.DebugDraw.UseDrawingBounds = Settings.UseCameraBounds;
            Draw.DebugDraw.DrawShapes = Settings.DrawShapes;
            Draw.DebugDraw.DrawJoints = Settings.DrawJoints;
            Draw.DebugDraw.DrawJointExtras = Settings.DrawJointExtras;
            Draw.DebugDraw.DrawAABBs = Settings.DrawAABBs;
            Draw.DebugDraw.DrawMass = Settings.DrawMass;
            Draw.DebugDraw.DrawContacts = Settings.DrawContactPoints;
            Draw.DebugDraw.DrawGraphColors = Settings.DrawGraphColors;
            Draw.DebugDraw.DrawContactNormals = Settings.DrawContactNormals;
            Draw.DebugDraw.DrawContactImpulses = Settings.DrawContactImpulses;
            Draw.DebugDraw.DrawFrictionImpulses = Settings.DrawFrictionImpulses;

            World.Draw(WorldId, Draw.DebugDraw);
            if (TimeStep > 0)
            {
                ++StepCount;
            }

            if (Settings.DrawCounters)
            {
                Counters s = World.GetCounters(WorldId);

                DrawString($"bodies/shapes/contacts/joints = {s.BodyCount}/{s.ShapeCount}/{s.ContactCount}/{s.JointCount}");
                DrawString($"islands/tasks = {s.IslandCount}/{s.TaskCount}");
                DrawString($"tree height static/movable = {s.StaticTreeHeight}/{s.TreeHeight}");

                var totalCount = 0;
                for (var i = 0; i < 12; ++i)
                {
                    totalCount += s.ColorCounts[i];
                }

                DrawString($"{string.Join('/', s.ColorCounts)}/[{totalCount}]");
                DrawString($"stack allocator size = {s.StackUsed / 1024} K");
                DrawString($"total allocation = {s.ByteCount / 1024} K");
            }

            // Track maximum profile times
            {
                var p = World.GetProfile(WorldId);
                MaxProfile.Step = Math.Max(MaxProfile.Step, p.Step);
                MaxProfile.Pairs = Math.Max(MaxProfile.Pairs, p.Pairs);
                MaxProfile.Collide = Math.Max(MaxProfile.Collide, p.Collide);
                MaxProfile.Solve = Math.Max(MaxProfile.Solve, p.Solve);
                MaxProfile.BuildIslands = Math.Max(MaxProfile.BuildIslands, p.BuildIslands);
                MaxProfile.SolveConstraints = Math.Max(MaxProfile.SolveConstraints, p.SolveConstraints);
                MaxProfile.PrepareTasks = Math.Max(MaxProfile.PrepareTasks, p.PrepareTasks);
                MaxProfile.SolverTasks = Math.Max(MaxProfile.SolverTasks, p.SolverTasks);
                MaxProfile.PrepareConstraints = Math.Max(MaxProfile.PrepareConstraints, p.PrepareConstraints);
                MaxProfile.IntegrateVelocities = Math.Max(MaxProfile.IntegrateVelocities, p.IntegrateVelocities);
                MaxProfile.WarmStart = Math.Max(MaxProfile.WarmStart, p.WarmStart);
                MaxProfile.SolveVelocities = Math.Max(MaxProfile.SolveVelocities, p.SolveVelocities);
                MaxProfile.IntegratePositions = Math.Max(MaxProfile.IntegratePositions, p.IntegratePositions);
                MaxProfile.RelaxVelocities = Math.Max(MaxProfile.RelaxVelocities, p.RelaxVelocities);
                MaxProfile.ApplyRestitution = Math.Max(MaxProfile.ApplyRestitution, p.ApplyRestitution);
                MaxProfile.StoreImpulses = Math.Max(MaxProfile.StoreImpulses, p.StoreImpulses);
                MaxProfile.FinalizeBodies = Math.Max(MaxProfile.FinalizeBodies, p.FinalizeBodies);
                MaxProfile.SleepIslands = Math.Max(MaxProfile.SleepIslands, p.SleepIslands);
                MaxProfile.SplitIslands = Math.Max(MaxProfile.SplitIslands, p.SplitIslands);
                MaxProfile.HitEvents = Math.Max(MaxProfile.HitEvents, p.HitEvents);
                MaxProfile.Broadphase = Math.Max(MaxProfile.Broadphase, p.Broadphase);
                MaxProfile.Continuous = Math.Max(MaxProfile.Continuous, p.Continuous);

                TotalProfile.Step += p.Step;
                TotalProfile.Pairs += p.Pairs;
                TotalProfile.Collide += p.Collide;
                TotalProfile.Solve += p.Solve;
                TotalProfile.BuildIslands += p.BuildIslands;
                TotalProfile.SolveConstraints += p.SolveConstraints;
                TotalProfile.PrepareTasks += p.PrepareTasks;
                TotalProfile.SolverTasks += p.SolverTasks;
                TotalProfile.PrepareConstraints += p.PrepareConstraints;
                TotalProfile.IntegrateVelocities += p.IntegrateVelocities;
                TotalProfile.WarmStart += p.WarmStart;
                TotalProfile.SolveVelocities += p.SolveVelocities;
                TotalProfile.IntegratePositions += p.IntegratePositions;
                TotalProfile.RelaxVelocities += p.RelaxVelocities;
                TotalProfile.ApplyRestitution += p.ApplyRestitution;
                TotalProfile.StoreImpulses += p.StoreImpulses;
                TotalProfile.FinalizeBodies += p.FinalizeBodies;
                TotalProfile.SleepIslands += p.SleepIslands;
                TotalProfile.SplitIslands += p.SplitIslands;
                TotalProfile.HitEvents += p.HitEvents;
                TotalProfile.Broadphase += p.Broadphase;
                TotalProfile.Continuous += p.Continuous;
            }

            // Profile
            if (Settings.DrawProfile)
            {
                Profile p = World.GetProfile(WorldId);

                Profile aveProfile = new();

                if (StepCount > 0)
                {
                    float scale = 1.0f / StepCount;
                    aveProfile.Step = scale * TotalProfile.Step;
                    aveProfile.Pairs = scale * TotalProfile.Pairs;
                    aveProfile.Collide = scale * TotalProfile.Collide;
                    aveProfile.Solve = scale * TotalProfile.Solve;
                    aveProfile.BuildIslands = scale * TotalProfile.BuildIslands;
                    aveProfile.SolveConstraints = scale * TotalProfile.SolveConstraints;
                    aveProfile.PrepareTasks = scale * TotalProfile.PrepareTasks;
                    aveProfile.SolverTasks = scale * TotalProfile.SolverTasks;
                    aveProfile.PrepareConstraints = scale * TotalProfile.PrepareConstraints;
                    aveProfile.IntegrateVelocities = scale * TotalProfile.IntegrateVelocities;
                    aveProfile.WarmStart = scale * TotalProfile.WarmStart;
                    aveProfile.SolveVelocities = scale * TotalProfile.SolveVelocities;
                    aveProfile.IntegratePositions = scale * TotalProfile.IntegratePositions;
                    aveProfile.RelaxVelocities = scale * TotalProfile.RelaxVelocities;
                    aveProfile.ApplyRestitution = scale * TotalProfile.ApplyRestitution;
                    aveProfile.StoreImpulses = scale * TotalProfile.StoreImpulses;
                    aveProfile.FinalizeBodies = scale * TotalProfile.FinalizeBodies;
                    aveProfile.SleepIslands = scale * TotalProfile.SleepIslands;
                    aveProfile.SplitIslands = scale * TotalProfile.SplitIslands;
                    aveProfile.HitEvents = scale * TotalProfile.HitEvents;
                    aveProfile.Broadphase = scale * TotalProfile.Broadphase;
                    aveProfile.Continuous = scale * TotalProfile.Continuous;
                }

                DrawString($"step [ave] (max) = {p.Step:F2} [{aveProfile.Step:F2}] ({MaxProfile.Step:F2})");
                DrawString($"pairs [ave] (max) = {p.Pairs:F2} [{aveProfile.Pairs:F2}] ({MaxProfile.Pairs:F2})");
                DrawString($"collide [ave] (max) = {p.Collide:F2} [{aveProfile.Collide:F2}] ({MaxProfile.Collide:F2})");
                DrawString($"solve [ave] (max) = {p.Solve:F2} [{aveProfile.Solve:F2}] ({MaxProfile.Solve:F2})");
                DrawString($"builds island [ave] (max) = {p.BuildIslands:F2} [{aveProfile.BuildIslands:F2}] ({MaxProfile.BuildIslands:F2})");
                DrawString($"solve constraints [ave] (max) = {p.SolveConstraints:F2} [{aveProfile.SolveConstraints:F2}] ({MaxProfile.SolveConstraints:F2})");
                DrawString($"prepare tasks [ave] (max) = {p.PrepareTasks:F2} [{aveProfile.PrepareTasks:F2}] ({MaxProfile.PrepareTasks:F2})");
                DrawString($"solver tasks [ave] (max) = {p.SolverTasks:F2} [{aveProfile.SolverTasks:F2}] ({MaxProfile.SolverTasks:F2})");
                DrawString($"prepare constraints [ave] (max) = {p.PrepareConstraints:F2} [{aveProfile.PrepareConstraints:F2}] ({MaxProfile.PrepareConstraints:F2})");
                DrawString($"integrate velocities [ave] (max) = {p.IntegrateVelocities:F2} [{aveProfile.IntegrateVelocities:F2}] ({MaxProfile.IntegrateVelocities:F2})");
                DrawString($"warm start [ave] (max) = {p.WarmStart:F2} [{aveProfile.WarmStart:F2}] ({MaxProfile.WarmStart:F2})");
                DrawString($"solve velocities [ave] (max) = {p.SolveVelocities:F2} [{aveProfile.SolveVelocities:F2}] ({MaxProfile.SolveVelocities:F2})");
                DrawString($"integrate positions [ave] (max) = {p.IntegratePositions:F2} [{aveProfile.IntegratePositions:F2}] ({MaxProfile.IntegratePositions:F2})");
                DrawString($"relax velocities [ave] (max) = {p.RelaxVelocities:F2} [{aveProfile.RelaxVelocities:F2}] ({MaxProfile.RelaxVelocities:F2})");
                DrawString($"apply restitution [ave] (max) = {p.ApplyRestitution:F2} [{aveProfile.ApplyRestitution:F2}] ({MaxProfile.ApplyRestitution:F2})");
                DrawString($"store impulses [ave] (max) = {p.StoreImpulses:F2} [{aveProfile.StoreImpulses:F2}] ({MaxProfile.StoreImpulses:F2})");
                DrawString($"finalize bodies [ave] (max) = {p.FinalizeBodies:F2} [{aveProfile.FinalizeBodies:F2}] ({MaxProfile.FinalizeBodies:F2})");
                DrawString($"sleep islands [ave] (max) = {p.SleepIslands:F2} [{aveProfile.SleepIslands:F2}] ({MaxProfile.SleepIslands:F2})");
                DrawString($"split islands [ave] (max) = {p.SplitIslands:F2} [{aveProfile.SplitIslands:F2}] ({MaxProfile.SplitIslands:F2})");
                DrawString($"hit events [ave] (max) = {p.HitEvents:F2} [{aveProfile.HitEvents:F2}] ({MaxProfile.HitEvents:F2})");
                DrawString($"broad-phase [ave] (max) = {p.Broadphase:F2} [{aveProfile.Broadphase:F2}] ({MaxProfile.Broadphase:F2})");
                DrawString($"continuous collision [ave] (max) = {p.Continuous:F2} [{aveProfile.Continuous:F2}] ({MaxProfile.Continuous:F2})");
            }

            OnRender();
            DrawWorld();
        }

        private void DrawWorld()
        {
            if (Settings.DrawShapes)
            { }

            if (Settings.DrawJoints)
            { }

            if (Settings.DrawAABBs)
            { }

            if (Settings.DrawContactPoints)
            { }
        }

        #endregion

        #region Keyboard Action

        public virtual void OnKeyDown(KeyInputEventArgs keyInput)
        { }

        public virtual void OnKeyUp(KeyInputEventArgs keyInput)
        { }

        #endregion Keyboard

        #region Mouse Action

        public virtual void MouseDown(Vec2 p, MouseInputEventArgs e)
        {
            if (!MouseJointId.IsNull)
            {
                return;
            }

            if (e.Button == MouseButton.Left)
            {
                // Make a small box.
                AABB box;
                Vec2 d = new(0.001f, 0.001f);
                box.LowerBound = B2Math.Sub(p, d);
                box.UpperBound = B2Math.Add(p, d);

                // Query the world for overlapping shapes.
                QueryContext queryContext = new(p, BodyId.NullId);
                World.OverlapAABB(WorldId, box, QueryFilter.DefaultQueryFilter(), QueryCallback, queryContext);

                if (!queryContext.BodyId.IsNull)
                {
                    BodyDef bodyDef = BodyDef.DefaultBodyDef();
                    GroundBodyId = Body.CreateBody(WorldId, bodyDef);

                    MouseJointDef mouseDef = MouseJointDef.DefaultMouseJointDef();
                    mouseDef.BodyIdA = GroundBodyId;
                    mouseDef.BodyIdB = queryContext.BodyId;
                    mouseDef.Target = p;
                    mouseDef.Hertz = 5.0f;
                    mouseDef.DampingRatio = 0.7f;
                    mouseDef.MaxForce = 1000.0f * Body.GetMass(queryContext.BodyId);
                    MouseJointId = Joint.CreateMouseJoint(WorldId, mouseDef);

                    Body.SetAwake(queryContext.BodyId, true);
                }
            }
        }

        public virtual void MouseUp(Vec2 p, MouseInputEventArgs e)
        {
            if (World.Joint_IsValid(MouseJointId) == false)
            {
                // The world or attached body was destroyed.
                MouseJointId = JointId.NullId;
            }

            if (!MouseJointId.IsNull && e.Button == MouseButton.Left)
            {
                Joint.DestroyJoint(MouseJointId);
                MouseJointId = JointId.NullId;

                Body.DestroyBody(GroundBodyId);
                GroundBodyId = BodyId.NullId;
            }
        }

        public virtual void MouseMove(Vec2 p, MouseMoveEventArgs e)
        {
            if (World.Joint_IsValid(MouseJointId) == false)
            {
                // The world or attached body was destroyed.
                MouseJointId = JointId.NullId;
            }

            if (!MouseJointId.IsNull)
            {
                MouseJointFunc.SetTarget(MouseJointId, p);
                BodyId bodyIdB = Joint.GetBodyB(MouseJointId);
                Body.SetAwake(bodyIdB, true);
            }
        }

        #endregion

        public void ShiftOrigin(Vec2 origin)
        {
            //World.ShiftOrigin(origin);
        }

        public virtual void JointDestroyed(Joint joint)
        { }

        public int RandomInt()
        {
            return B2Random.Shared.RandomInt();
        }

        /// Random number in range [-1,1]
        public float RandomFloat()
        {
            return B2Random.Shared.RandomFloat();
        }

        /// Random floating point number in range [lo, hi]
        public float RandomFloat(float lo, float hi)
        {
            return B2Random.Shared.RandomFloat(lo, hi);
        }

        public struct ContactPoint
        {
            public Vec2 Normal;

            public Vec2 Position;

            public float NormalImpulse;

            public float TangentImpulse;

            public float Separation;
        }

        public virtual void Dispose()
        {
            World.DestroyWorld(WorldId);
            WorldId = WorldId.NullId;

            // World?.Dispose();
            // World = null;
        }

        public bool QueryCallback(ShapeId shapeId, object context)
        {
            QueryContext queryContext = (QueryContext)context;
            BodyId bodyId = Shape.GetBody(shapeId);
            BodyType bodyType = Body.GetType(bodyId);
            if (bodyType != BodyType.DynamicBody)
            {
                // continue query
                return true;
            }

            bool overlap = Shape.TestPoint(shapeId, queryContext.Point);
            if (overlap)
            {
                // found shape
                queryContext.BodyId = bodyId;
                return false;
            }

            return true;
        }
    }

    public class QueryContext
    {
        public Vec2 Point;

        public BodyId BodyId = BodyId.NullId;

        public QueryContext()
        { }

        public QueryContext(Vec2 point, BodyId bodyId)
        {
            this.Point = point;
            this.BodyId = bodyId;
        }
    }
}