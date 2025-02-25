using System;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;

namespace Box2DSharp
{
    /// <summary>
    /// 解算器
    /// </summary>
    public class Solver
    {
        public static void Pause()
        {
            Thread.Sleep(1);
        }

        public static void Yield()
        {
            Thread.Yield();
        }

        /// <summary>
        /// Integrate velocities and apply damping
        /// [任务]速度积分，计算阻尼
        /// </summary>
        /// <param name="startIndex"></param>
        /// <param name="endIndex"></param>
        /// <param name="context"></param>
        private static void IntegrateVelocitiesTask(int startIndex, int endIndex, StepContext context)
        {
            var states = context.States;
            var sims = context.Sims;

            Vec2 gravity = context.World.Gravity;
            float h = context.H;
            float maxLinearSpeed = context.MaxLinearVelocity;
            float maxAngularSpeed = Core.MaxRotation * context.InvDt;
            float maxLinearSpeedSquared = maxLinearSpeed * maxLinearSpeed;
            float maxAngularSpeedSquared = maxAngularSpeed * maxAngularSpeed;

            for (int i = startIndex; i < endIndex; ++i)
            {
                BodySim sim = sims[i];
                ref BodyState state = ref states[i];

                Vec2 v = state.LinearVelocity;
                float w = state.AngularVelocity;

                // Apply forces, torque, gravity, and damping 【计算受力、扭矩、重力、阻尼】
                // Apply damping.
                // Differential equation: dv/dt + c * v = 0 【阻尼微分方程，表示速度的变化量，c是阻尼系数，v是速度】
                // Solution: v(t) = v0 * exp(-c * t) 【求积分得到t时间的速度，v0是初始速度，exp(-c*t)是自然对数的-c*t次方，即e^(-c*t)】
                // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v(t) * exp(-c * dt) 【时间步进，简化得到t+dt时间的速度是t时间速度*e^(-c*dt)】
                // v2 = exp(-c * dt) * v1【v1是t时间速度，v2是t+dt时间速度】
                // Pade approximation: 【帕德近似，得到简化方程】
                // v2 = v1 * 1 / (1 + c * dt) 
                float linearDamping = 1.0f / (1.0f + h * sim.LinearDamping);   // c阻尼系数：LinearDamping，线速度阻尼，dt即h，子步时间分片，先计算(1 / (1 + c * dt))部分
                float angularDamping = 1.0f / (1.0f + h * sim.AngularDamping); // c阻尼系数：AngularDamping，角速度阻尼，dt即h，子步时间分片

                // 受力速度计算，当前时间片速度变化量 = (时间片*质量倒数) * (外力+(质量*重力比例)*重力矢量) 
                Vec2 linearVelocityDelta = B2Math.MulSV(h * sim.InvMass, B2Math.MulAdd(sim.Force, sim.Mass * sim.GravityScale, gravity));

                // 子步
                // 刚体定轴转动定律，合力矩=转动惯量x角加速度 => 角加速度=合力矩/转动惯量 => 角加速度=转动惯量倒数*合力矩 => 当前子步的受力角速度=角加速度*时间分片
                float angularVelocityDelta = h * sim.InvInertia * sim.Torque;

                v = B2Math.MulAdd(linearVelocityDelta, linearDamping, v); // v * linearDamping得到应用阻尼后的速度，加上重力影响的速度变化，得到本子步的最终线速度
                w = angularVelocityDelta + angularDamping * w;            // 子步最终角速度 = 受力角速度 + 角阻尼 * 上一步角速度

                // Clamp to max linear speed 限制线速度
                if (B2Math.Dot(v, v) > maxLinearSpeedSquared)
                {
                    float ratio = maxLinearSpeed / v.Length;
                    v = B2Math.MulSV(ratio, v);
                    sim.IsSpeedCapped = true;
                }

                // Clamp to max angular speed // 限制角速度
                if (w * w > maxAngularSpeedSquared && sim.AllowFastRotation == false)
                {
                    float ratio = maxAngularSpeed / Math.Abs(w);
                    w *= ratio;
                    sim.IsSpeedCapped = true;
                }

                state.LinearVelocity = v;
                state.AngularVelocity = w;
            }
        }

        /// <summary>
        /// [任务]准备关节
        /// </summary>
        /// <param name="startIndex"></param>
        /// <param name="endIndex"></param>
        /// <param name="context"></param>
        private static void PrepareJointsTask(int startIndex, int endIndex, StepContext context)
        {
            var joints = context.Joints;

            for (int i = startIndex; i < endIndex; ++i)
            {
                var joint = joints[i];
                Joint.PrepareJoint(joint, context);
            }
        }

        /// <summary>
        /// [任务]热启动关节
        /// </summary>
        /// <param name="startIndex"></param>
        /// <param name="endIndex"></param>
        /// <param name="context"></param>
        /// <param name="colorIndex"></param>
        private static void WarmStartJointsTask(int startIndex, int endIndex, StepContext context, int colorIndex)
        {
            GraphColor color = context.Graph.Colors[colorIndex];
            Span<JointSim> joints = color.Joints;
            Debug.Assert(0 <= startIndex && startIndex < color.Joints.Count);
            Debug.Assert(startIndex <= endIndex && endIndex <= color.Joints.Count);

            for (int i = startIndex; i < endIndex; ++i)
            {
                JointSim joint = joints[i];
                Joint.WarmStartJoint(joint, context);
            }
        }

        /// <summary>
        /// [任务]解算关节
        /// </summary>
        /// <param name="startIndex"></param>
        /// <param name="endIndex"></param>
        /// <param name="context"></param>
        /// <param name="colorIndex"></param>
        /// <param name="useBias"></param>
        private static void SolveJointsTask(int startIndex, int endIndex, StepContext context, int colorIndex, bool useBias)
        {
            GraphColor color = context.Graph.Colors[colorIndex];
            Span<JointSim> joints = color.Joints;
            Debug.Assert(0 <= startIndex && startIndex < color.Joints.Count);
            Debug.Assert(startIndex <= endIndex && endIndex <= color.Joints.Count);

            for (int i = startIndex; i < endIndex; ++i)
            {
                JointSim joint = joints[i];
                Joint.SolveJoint(joint, context, useBias);
            }
        }

        /// <summary>
        /// [任务]位置积分
        /// </summary>
        /// <param name="startIndex"></param>
        /// <param name="endIndex"></param>
        /// <param name="context"></param>
        private static void IntegratePositionsTask(int startIndex, int endIndex, StepContext context)
        {
            var states = context.States;
            float h = context.H;

            Debug.Assert(startIndex <= endIndex);

            for (int i = startIndex; i < endIndex; ++i)
            {
                ref BodyState state = ref states[i];
                state.DeltaRotation = B2Math.IntegrateRotation(state.DeltaRotation, h * state.AngularVelocity);
                state.DeltaPosition = B2Math.MulAdd(state.DeltaPosition, h, state.LinearVelocity);
            }
        }

        /// <summary>
        /// [任务]终结刚体任务
        /// </summary>
        /// <param name="startIndex"></param>
        /// <param name="endIndex"></param>
        /// <param name="threadIndex"></param>
        /// <param name="context"></param>
        private static void FinalizeBodiesTask(int startIndex, int endIndex, int threadIndex, object context)
        {
            StepContext stepContext = (StepContext)context;
            World world = stepContext.World;
            bool enableSleep = world.EnableSleep;
            var states = stepContext.States;
            var sims = stepContext.Sims;
            var bodies = world.BodyArray;
            float timeStep = stepContext.Dt;
            float invTimeStep = stepContext.InvDt;

            ushort worldId = world.WorldId;
            var moveEvents = world.BodyMoveEventArray;

            var islands = world.IslandArray;

            var enlargedSimBitSet = world.TaskContextArray[threadIndex].EnlargedSimBitSet;
            var awakeIslandBitSet = world.TaskContextArray[threadIndex].AwakeIslandBitSet;
            TaskContext taskContext = world.TaskContextArray[threadIndex];

            bool enableContinuous = world.EnableContinuous;

            float speculativeDistance = Core.SpeculativeDistance;
            float aabbMargin = Core.b2_aabbMargin;

            Debug.Assert(startIndex <= endIndex);

            for (int simIndex = startIndex; simIndex < endIndex; ++simIndex)
            {
                ref BodyState state = ref states[simIndex];
                BodySim sim = sims[simIndex];

                Vec2 v = state.LinearVelocity;
                float w = state.AngularVelocity;

                Debug.Assert(B2Math.Vec2_IsValid(v));
                Debug.Assert(B2Math.IsValid(w));

                sim.Center = B2Math.Add(sim.Center, state.DeltaPosition);
                sim.Transform.Q = B2Math.NormalizeRot(B2Math.MulRot(state.DeltaRotation, sim.Transform.Q));

                // Use the velocity of the farthest point on the body to account for rotation.
                float maxVelocity = v.Length + Math.Abs(w) * sim.MaxExtent;

                // Sleep needs to observe position correction as well as true velocity.
                float maxDeltaPosition = state.DeltaPosition.Length + Math.Abs(state.DeltaRotation.S) * sim.MaxExtent;

                // Position correction is not as important for sleep as true velocity.
                float positionSleepFactor = 0.5f;

                float sleepVelocity = Math.Max(maxVelocity, positionSleepFactor * invTimeStep * maxDeltaPosition);

                // reset state deltas
                state.DeltaPosition = Vec2.Zero;
                state.DeltaRotation = Rot.Identity;

                sim.Transform.P = B2Math.Sub(sim.Center, B2Math.RotateVector(sim.Transform.Q, sim.LocalCenter));

                // cache miss here, however I need the shape list below
                var body = bodies[sim.BodyId];
                body.BodyMoveIndex = simIndex;
                moveEvents[simIndex].Transform = sim.Transform;
                moveEvents[simIndex].BodyId = new(sim.BodyId + 1, worldId, body.Revision);
                moveEvents[simIndex].UserData = body.UserData;
                moveEvents[simIndex].FellAsleep = false;

                // reset applied force and torque
                sim.Force = Vec2.Zero;
                sim.Torque = 0.0f;

                body.IsSpeedCapped = sim.IsSpeedCapped;
                sim.IsSpeedCapped = false;

                sim.IsFast = false;

                // 世界或刚体不允许休眠、休眠速度超过阈值时
                if (enableSleep == false || body.EnableSleep == false || sleepVelocity > body.SleepThreshold)
                {
                    // Body is not sleepy
                    body.SleepTime = 0.0f;

                    const float SafetyFactor = 0.5f;
                    if (body.Type == BodyType.DynamicBody && enableContinuous && maxVelocity * timeStep > SafetyFactor * sim.MinExtent)
                    {
                        // Store in fast array for the continuous collision stage
                        // This is deterministic because the order of TOI sweeps doesn't matter
                        if (sim.IsBullet)
                        {
                            int bulletIndex = Interlocked.Increment(ref stepContext.BulletBodyCount) - 1;
                            stepContext.BulletBodies[bulletIndex] = simIndex;
                        }
                        else
                        {
                            int fastIndex = Interlocked.Increment(ref stepContext.FastBodyCount) - 1;
                            stepContext.FastBodies[fastIndex] = simIndex;
                        }

                        sim.IsFast = true;
                    }
                    else
                    {
                        // Body is safe to advance
                        sim.Center0 = sim.Center;
                        sim.Rotation0 = sim.Transform.Q;
                    }
                }
                else
                {
                    // Body is safe to advance and is falling asleep
                    sim.Center0 = sim.Center;
                    sim.Rotation0 = sim.Transform.Q;
                    body.SleepTime += timeStep;
                }

                // Any single body in an island can keep it awake
                islands.CheckIndex(body.IslandId);
                var island = islands[body.IslandId];
                if (body.SleepTime < Core.TimeToSleep)
                {
                    // keep island awake
                    int islandIndex = island.LocalIndex;
                    awakeIslandBitSet.SetBit(islandIndex);
                }
                else if (island.ConstraintRemoveCount > 0)
                {
                    // body wants to sleep but its island needs splitting first
                    if (body.SleepTime > taskContext.SplitSleepTime)
                    {
                        // pick the sleepiest candidate
                        taskContext.SplitIslandId = body.IslandId;
                        taskContext.SplitSleepTime = body.SleepTime;
                    }
                }

                // Update shapes AABBs
                Transform transform = sim.Transform;
                bool isFast = sim.IsFast;
                int shapeId = body.HeadShapeId;
                while (shapeId != Core.NullIndex)
                {
                    Shape shape = world.ShapeArray[shapeId];

                    Debug.Assert(shape.IsFast == false);

                    if (isFast)
                    {
                        // The AABB is updated after continuous collision.
                        // Add to moved shapes regardless of AABB changes.
                        shape.IsFast = true;

                        // Bit-set to keep the move array sorted
                        enlargedSimBitSet.SetBit(simIndex);
                    }
                    else
                    {
                        AABB aabb = shape.ComputeShapeAABB(transform);
                        aabb.LowerBound.X -= speculativeDistance;
                        aabb.LowerBound.Y -= speculativeDistance;
                        aabb.UpperBound.X += speculativeDistance;
                        aabb.UpperBound.Y += speculativeDistance;
                        shape.AABB = aabb;

                        Debug.Assert(shape.EnlargedAABB == false);

                        if (B2Math.AABB_Contains(shape.FatAABB, aabb) == false)
                        {
                            AABB fatAABB;
                            fatAABB.LowerBound.X = aabb.LowerBound.X - aabbMargin;
                            fatAABB.LowerBound.Y = aabb.LowerBound.Y - aabbMargin;
                            fatAABB.UpperBound.X = aabb.UpperBound.X + aabbMargin;
                            fatAABB.UpperBound.Y = aabb.UpperBound.Y + aabbMargin;
                            shape.FatAABB = fatAABB;

                            shape.EnlargedAABB = true;

                            // Bit-set to keep the move array sorted
                            enlargedSimBitSet.SetBit(simIndex);
                        }
                    }

                    shapeId = shape.NextShapeId;
                }
            }
        }

        /// <summary>
        /// 执行区块任务
        /// </summary>
        /// <param name="stage"></param>
        /// <param name="context"></param>
        /// <param name="block"></param>
        private static void ExecuteBlock(SolverStage stage, StepContext context, SolverBlock block)
        {
            SolverStageType stageType = stage.Type;
            SolverBlockType blockType = (SolverBlockType)block.BlockType;
            int startIndex = block.StartIndex;
            int endIndex = startIndex + block.Count;

            switch (stageType)
            {
            case SolverStageType.PrepareJoints:
                PrepareJointsTask(startIndex, endIndex, context);
                break;

            case SolverStageType.PrepareContacts:
                ContactSolver.PrepareContactsTask(startIndex, endIndex, context);
                break;

            case SolverStageType.IntegrateVelocities:
                IntegrateVelocitiesTask(startIndex, endIndex, context);
                break;

            case SolverStageType.WarmStart:
                if (context.World.EnableWarmStarting)
                {
                    if (blockType == SolverBlockType.GraphContactBlock)
                    {
                        ContactSolver.WarmStartContactsTask(startIndex, endIndex, context, stage.ColorIndex);
                    }
                    else if (blockType == SolverBlockType.GraphJointBlock)
                    {
                        WarmStartJointsTask(startIndex, endIndex, context, stage.ColorIndex);
                    }
                }

                break;

            case SolverStageType.Solve:
                if (blockType == SolverBlockType.GraphContactBlock)
                {
                    ContactSolver.SolveContactsTask(startIndex, endIndex, context, stage.ColorIndex, true);
                }
                else if (blockType == SolverBlockType.GraphJointBlock)
                {
                    SolveJointsTask(startIndex, endIndex, context, stage.ColorIndex, true);
                }

                break;

            case SolverStageType.IntegratePositions:
                IntegratePositionsTask(startIndex, endIndex, context);
                break;

            case SolverStageType.Relax:
                if (blockType == SolverBlockType.GraphContactBlock)
                {
                    ContactSolver.SolveContactsTask(startIndex, endIndex, context, stage.ColorIndex, false);
                }
                else if (blockType == SolverBlockType.GraphJointBlock)
                {
                    SolveJointsTask(startIndex, endIndex, context, stage.ColorIndex, false);
                }

                break;

            case SolverStageType.Restitution:
                if (blockType == SolverBlockType.GraphContactBlock)
                {
                    ContactSolver.ApplyRestitutionTask(startIndex, endIndex, context, stage.ColorIndex);
                }

                break;

            case SolverStageType.StoreImpulses:
                ContactSolver.StoreImpulsesTask(startIndex, endIndex, context);
                break;
            }
        }

        public static int GetWorkerStartIndex(int workerIndex, int blockCount, int workerCount)
        {
            if (blockCount <= workerCount)
            {
                return workerIndex < blockCount ? workerIndex : Core.NullIndex;
            }

            int blocksPerWorker = blockCount / workerCount;
            int remainder = blockCount - blocksPerWorker * workerCount;
            return blocksPerWorker * workerIndex + Math.Min(remainder, workerIndex);
        }

        public static void ExecuteStage(SolverStage stage, StepContext context, int previousSyncIndex, int syncIndex, int workerIndex)
        {
            int completedCount = 0;
            var blocks = stage.Blocks;
            int blockCount = stage.BlockCount;

            int expectedSyncIndex = previousSyncIndex;

            int startIndex = GetWorkerStartIndex(workerIndex, blockCount, context.WorkerCount);
            if (startIndex == Core.NullIndex)
            {
                return;
            }

            Debug.Assert(0 <= startIndex && startIndex < blockCount);

            int blockIndex = startIndex;

            // Caution: this can change expectedSyncIndex
            while (Interlocked.CompareExchange(ref blocks[blockIndex].SyncIndex, syncIndex, expectedSyncIndex) == expectedSyncIndex)
            {
                Debug.Assert(stage.Type != SolverStageType.PrepareContacts || syncIndex < 2);

                Debug.Assert(completedCount < blockCount);

                ExecuteBlock(stage, context, blocks[blockIndex]);

                completedCount += 1;
                blockIndex += 1;
                if (blockIndex >= blockCount)
                {
                    // Keep looking for work
                    blockIndex = 0;
                }

                //expectedSyncIndex = previousSyncIndex;
            }

            // Search backwards for blocks
            blockIndex = startIndex - 1;
            while (true)
            {
                if (blockIndex < 0)
                {
                    blockIndex = blockCount - 1;
                }

                //expectedSyncIndex = previousSyncIndex;

                // Caution: this can change expectedSyncIndex
                if (Interlocked.CompareExchange(ref blocks[blockIndex].SyncIndex, syncIndex, expectedSyncIndex) != expectedSyncIndex)
                {
                    break;
                }

                ExecuteBlock(stage, context, blocks[blockIndex]);
                completedCount += 1;
                blockIndex -= 1;
            }

            Interlocked.Add(ref stage.CompletionCount, completedCount);
        }

        private static void ExecuteMainStage(SolverStage stage, StepContext context, uint syncBits)
        {
            int blockCount = stage.BlockCount;
            if (blockCount == 0)
            {
                return;
            }

            if (blockCount == 1)
            {
                ExecuteBlock(stage, context, stage.Blocks[0]);
            }
            else
            {
                InterlockedHelper.Exchange(ref context.AtomicSyncBits, syncBits);

                int syncIndex = (int)((syncBits >> 16) & 0xFFFF);
                Debug.Assert(syncIndex > 0);
                int previousSyncIndex = syncIndex - 1;

                ExecuteStage(stage, context, previousSyncIndex, syncIndex, 0);

                // todo consider using the cycle counter as well
                while (Interlocked.CompareExchange(ref stage.CompletionCount, 0, 0) != blockCount)
                {
                    Pause();
                }

                Interlocked.Exchange(ref stage.CompletionCount, 0);
            }
        }

        // This should not use the thread index because thread 0 can be called twice by enkiTS.
        public static void SolverTask(WorkerContext workerContext)
        {
            int workerIndex = workerContext.WorkerIndex;
            StepContext context = workerContext.Context;
            int activeColorCount = context.ActiveColorCount;
            var stages = context.Stages;
            ref Profile profile = ref context.World.Profile;

            if (workerIndex == 0)
            {
                // Main thread synchronizes the workers and does work itself.
                //
                // Stages are re-used by loops so that I don't need more stages for large iteration counts.
                // The sync indices grow monotonically for the body/graph/constraint groupings because they share solver blocks.
                // The stage index and sync indices are combined in to sync bits for atomic synchronization.
                // The workers need to compute the previous sync index for a given stage so that CAS works correctly. This
                // setup makes this easy to do.

                /*
                b2_stagePrepareJoints,
                b2_stagePrepareContacts,
                b2_stageIntegrateVelocities,
                b2_stageWarmStart,
                b2_stageSolve,
                b2_stageIntegratePositions,
                b2_stageRelax,
                b2_stageRestitution,
                b2_stageStoreImpulses
                */

                var t1 = Stopwatch.GetTimestamp();

                int bodySyncIndex = 1;
                int stageIndex = 0;

                // This stage loops over all awake joints
                uint jointSyncIndex = 1;
                uint syncBits = (jointSyncIndex << 16) | (uint)stageIndex;
                Debug.Assert(stages[stageIndex].Type == SolverStageType.PrepareJoints);
                ExecuteMainStage(stages[stageIndex], context, syncBits);
                stageIndex += 1;
                jointSyncIndex += 1;

                // This stage loops over all contact constraints
                uint contactSyncIndex = 1;
                syncBits = (contactSyncIndex << 16) | (uint)stageIndex;
                Debug.Assert(stages[stageIndex].Type == SolverStageType.PrepareContacts);
                ExecuteMainStage(stages[stageIndex], context, syncBits);
                stageIndex += 1;
                contactSyncIndex += 1;

                int graphSyncIndex = 1;

                // Single-threaded overflow work. These constraints don't fit in the graph coloring.
                Joint.PrepareOverflowJoints(context);
                ContactSolver.PrepareOverflowContacts(context);

                profile.PrepareConstraints += (float)StopwatchHelper.GetElapsedTime(t1).TotalMilliseconds;

                int subStepCount = context.SubStepCount;
                for (int i = 0; i < subStepCount; ++i)
                {
                    // stage index restarted each iteration
                    // syncBits still increases monotonically because the upper bits increase each iteration
                    t1 = Stopwatch.GetTimestamp();
                    int iterStageIndex = stageIndex;

                    // integrate velocities
                    syncBits = (uint)((bodySyncIndex << 16) | iterStageIndex);
                    Debug.Assert(stages[iterStageIndex].Type == SolverStageType.IntegrateVelocities);
                    ExecuteMainStage(stages[iterStageIndex], context, syncBits);
                    iterStageIndex += 1;
                    bodySyncIndex += 1;

                    profile.IntegrateVelocities += (float)StopwatchHelper.GetElapsedTime(t1).TotalMilliseconds;

                    // warm start constraints
                    t1 = Stopwatch.GetTimestamp();
                    Joint.WarmStartOverflowJoints(context);
                    ContactSolver.WarmStartOverflowContacts(context);

                    for (int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex)
                    {
                        syncBits = (uint)((graphSyncIndex << 16) | iterStageIndex);
                        Debug.Assert(stages[iterStageIndex].Type == SolverStageType.WarmStart);
                        ExecuteMainStage(stages[iterStageIndex], context, syncBits);
                        iterStageIndex += 1;
                    }

                    graphSyncIndex += 1;

                    profile.WarmStart += (float)StopwatchHelper.GetElapsedTime(t1).TotalMilliseconds;

                    // solve constraints
                    t1 = Stopwatch.GetTimestamp();
                    bool useBias = true;
                    Joint.SolveOverflowJoints(context, useBias);
                    ContactSolver.SolveOverflowContacts(context, useBias);

                    for (int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex)
                    {
                        syncBits = (uint)((graphSyncIndex << 16) | iterStageIndex);
                        Debug.Assert(stages[iterStageIndex].Type == SolverStageType.Solve);
                        ExecuteMainStage(stages[iterStageIndex], context, syncBits);
                        iterStageIndex += 1;
                    }

                    graphSyncIndex += 1;

                    profile.SolveVelocities += (float)StopwatchHelper.GetElapsedTime(t1).TotalMilliseconds;

                    // integrate positions
                    t1 = Stopwatch.GetTimestamp();
                    Debug.Assert(stages[iterStageIndex].Type == SolverStageType.IntegratePositions);
                    syncBits = (uint)((bodySyncIndex << 16) | iterStageIndex);
                    ExecuteMainStage(stages[iterStageIndex], context, syncBits);
                    iterStageIndex += 1;
                    bodySyncIndex += 1;

                    profile.IntegratePositions += (float)StopwatchHelper.GetElapsedTime(t1).TotalMilliseconds;

                    // relax constraints
                    t1 = Stopwatch.GetTimestamp();
                    useBias = false;
                    Joint.SolveOverflowJoints(context, useBias);
                    ContactSolver.SolveOverflowContacts(context, useBias);

                    for (int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex)
                    {
                        syncBits = (uint)((graphSyncIndex << 16) | iterStageIndex);
                        Debug.Assert(stages[iterStageIndex].Type == SolverStageType.Relax);
                        ExecuteMainStage(stages[iterStageIndex], context, syncBits);
                        iterStageIndex += 1;
                    }

                    graphSyncIndex += 1;

                    profile.RelaxVelocities += (float)StopwatchHelper.GetElapsedTime(t1).TotalMilliseconds;
                }

                // advance the stage according to the sub-stepping tasks just completed
                // integrate velocities / warm start / solve / integrate positions / relax
                stageIndex += 1 + activeColorCount + activeColorCount + 1 + activeColorCount;

                // Restitution
                t1 = Stopwatch.GetTimestamp();
                {
                    ContactSolver.ApplyOverflowRestitution(context);

                    int iterStageIndex = stageIndex;
                    for (int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex)
                    {
                        syncBits = (uint)((graphSyncIndex << 16) | iterStageIndex);
                        Debug.Assert(stages[iterStageIndex].Type == SolverStageType.Restitution);
                        ExecuteMainStage(stages[iterStageIndex], context, syncBits);
                        iterStageIndex += 1;
                    }

                    // graphSyncIndex += 1;
                    stageIndex += activeColorCount;
                }

                profile.ApplyRestitution += (float)StopwatchHelper.GetElapsedTime(t1).TotalMilliseconds;

                t1 = Stopwatch.GetTimestamp();
                ContactSolver.StoreOverflowImpulses(context);

                syncBits = (contactSyncIndex << 16) | (uint)stageIndex;
                Debug.Assert(stages[stageIndex].Type == SolverStageType.StoreImpulses);
                ExecuteMainStage(stages[stageIndex], context, syncBits);
                profile.StoreImpulses += (float)StopwatchHelper.GetElapsedTime(t1).TotalMilliseconds;

                // Signal workers to finish
                InterlockedHelper.Exchange(ref context.AtomicSyncBits, uint.MaxValue);

                Debug.Assert(stageIndex + 1 == context.StageCount);
                return;
            }

            // Worker spins and waits for work
            uint lastSyncBits = 0;

            // uint64_t maxSpinTime = 10;
            while (true)
            {
                // Spin until main thread bumps changes the sync bits. This can waste significant time overall, but it is necessary for
                // parallel simulation with graph coloring.
                uint syncBits;
                int spinCount = 0;
                while ((syncBits = InterlockedHelper.CompareExchange(ref context.AtomicSyncBits, 0, 0)) == lastSyncBits)
                {
                    if (spinCount > 5)
                    {
                        Yield();
                        spinCount = 0;
                    }
                    else
                    {
                        // Using the cycle counter helps to account for variation in mm_pause timing across different
                        // CPUs. However, this is X64 only.
                        // uint64_t prev = __rdtsc();
                        // do
                        //{
                        //	b2Pause();
                        //}
                        // while ((__rdtsc() - prev) < maxSpinTime);
                        // maxSpinTime += 10;
                        Pause();
                        spinCount += 1;
                    }
                }

                if (syncBits == uint.MaxValue)
                {
                    // sentinel hit
                    break;
                }

                int stageIndex = (int)(syncBits & 0xFFFF);
                Debug.Assert(stageIndex < context.StageCount);

                int syncIndex = (int)((syncBits >> 16) & 0xFFFF);
                Debug.Assert(syncIndex > 0);

                int previousSyncIndex = syncIndex - 1;

                SolverStage stage = stages[stageIndex];
                ExecuteStage(stage, context, previousSyncIndex, syncIndex, workerIndex);

                lastSyncBits = syncBits;
            }
        }

        public class ContinuousContext
        {
            public World World = null!;

            public BodySim FastBodySim = null!;

            public Shape FastShape = null!;

            public Vec2 Centroid1;

            public Vec2 Centroid2;

            public Sweep Sweep;

            public float Fraction;
        }

        /// <summary>
        /// todo this may lead to pauses for scenarios where pre-solve would disable collision
        /// </summary>
        private static readonly TreeQueryCallbackFcn _continuousQueryCallback = (int proxyId, int shapeId, object context) =>
        {
            var continuousContext = (ContinuousContext)context;
            Shape fastShape = continuousContext.FastShape;
            BodySim fastBodySim = continuousContext.FastBodySim;

            // Skip same shape
            if (shapeId == fastShape.Id)
            {
                return true;
            }

            World world = continuousContext.World;

            world.ShapeArray.CheckId(shapeId);
            Shape shape = world.ShapeArray[shapeId];

            // Skip same body
            if (shape.BodyId == fastShape.BodyId)
            {
                return true;
            }

            // Skip filtered shapes
            bool canCollide = Contact.ShouldShapesCollide(fastShape.Filter, shape.Filter);
            if (canCollide == false)
            {
                return true;
            }

            // Skip sensors
            if (shape.IsSensor == true)
            {
                return true;
            }

            world.BodyArray.CheckIndex(shape.BodyId);
            Body body = world.BodyArray[shape.BodyId];
            BodySim bodySim = Body.GetBodySim(world, body);
            Debug.Assert(body.Type == BodyType.StaticBody || fastBodySim.IsBullet);

            // Skip bullets
            if (bodySim.IsBullet)
            {
                return true;
            }

            // Skip filtered bodies
            world.BodyArray.CheckIndex(fastBodySim.BodyId);
            Body fastBody = world.BodyArray[fastBodySim.BodyId];
            canCollide = Body.ShouldBodiesCollide(world, fastBody, body);
            if (canCollide == false)
            {
                return true;
            }

            // Custom user filtering
            var customFilterFcn = world.CustomFilterFcn;
            if (customFilterFcn != null)
            {
                ShapeId idA = (shape.Id + 1, world.WorldId, shape.Revision);
                ShapeId idB = (fastShape.Id + 1, world.WorldId, fastShape.Revision);
                canCollide = customFilterFcn(idA, idB, world.CustomFilterContext);
                if (canCollide == false)
                {
                    return true;
                }
            }

            // Prevent pausing on chain segment junctions
            if (shape.Type == ShapeType.ChainSegmentShape)
            {
                Transform transform = bodySim.Transform;
                Vec2 p1 = B2Math.TransformPoint(transform, shape.Union.ChainSegment.Segment.Point1);
                Vec2 p2 = B2Math.TransformPoint(transform, shape.Union.ChainSegment.Segment.Point2);
                Vec2 e = B2Math.Sub(p2, p1);
                Vec2 c1 = continuousContext.Centroid1;
                Vec2 c2 = continuousContext.Centroid2;
                float offset1 = B2Math.Cross(B2Math.Sub(c1, p1), e);
                float offset2 = B2Math.Cross(B2Math.Sub(c2, p1), e);

                if (offset1 < 0.0f || offset2 > 0.0f)
                {
                    // Started behind or finished in front
                    return true;
                }
            }

            TOIInput input;
            input.ProxyA = Shape.MakeShapeDistanceProxy(shape);
            input.ProxyB = Shape.MakeShapeDistanceProxy(fastShape);
            input.SweepA = BodySim.MakeSweep(bodySim);
            input.SweepB = continuousContext.Sweep;
            input.TMax = continuousContext.Fraction;

            TOIOutput output = DistanceFunc.TimeOfImpact(input);
            if (0.0f < output.T && output.T < continuousContext.Fraction)
            {
                continuousContext.Fraction = output.T;
            }
            else if (0.0f == output.T)
            {
                // fallback to TOI of a small circle around the fast shape centroid
                Vec2 centroid = Shape.GetShapeCentroid(fastShape);
                input.ProxyB = DistanceFunc.MakeProxy(centroid, Core.SpeculativeDistance);
                output = DistanceFunc.TimeOfImpact(input);
                if (0.0f < output.T && output.T < continuousContext.Fraction)
                {
                    continuousContext.Fraction = output.T;
                }
            }

            return true;
        };

        // Continuous collision of dynamic versus static
        private static void SolveContinuous(World world, int bodySimIndex)
        {
            SolverSet awakeSet = world.SolverSetArray[SolverSetType.AwakeSet];
            Debug.Assert(0 <= bodySimIndex && bodySimIndex < awakeSet.Sims.Count);
            BodySim fastBodySim = awakeSet.Sims.Data[bodySimIndex];
            Debug.Assert(fastBodySim.IsFast);

            var shapes = world.ShapeArray;

            Sweep sweep = BodySim.MakeSweep(fastBodySim);

            Transform xf1;
            xf1.Q = sweep.Q1;
            xf1.P = B2Math.Sub(sweep.C1, B2Math.RotateVector(sweep.Q1, sweep.LocalCenter));

            Transform xf2;
            xf2.Q = sweep.Q2;
            xf2.P = B2Math.Sub(sweep.C2, B2Math.RotateVector(sweep.Q2, sweep.LocalCenter));

            var staticTree = world.BroadPhase.Trees[(int)BodyType.StaticBody];
            var kinematicTree = world.BroadPhase.Trees[(int)BodyType.KinematicBody];
            var dynamicTree = world.BroadPhase.Trees[(int)BodyType.DynamicBody];

            ContinuousContext context = B2ObjectPool<ContinuousContext>.Shared.Get();
            context.World = world;
            context.Sweep = sweep;
            context.FastBodySim = fastBodySim;
            context.Fraction = 1.0f;

            bool isBullet = fastBodySim.IsBullet;

            // todo consider moving shape list to body sim
            world.BodyArray.CheckIndex(fastBodySim.BodyId);
            var fastBody = world.BodyArray[fastBodySim.BodyId];
            int shapeId = fastBody.HeadShapeId;
            while (shapeId != Core.NullIndex)
            {
                shapes.CheckId(shapeId);
                Shape fastShape = shapes[shapeId];
                Debug.Assert(fastShape.IsFast == true);

                shapeId = fastShape.NextShapeId;

                // Clear flag (keep set on body)
                fastShape.IsFast = false;

                context.FastShape = fastShape;
                context.Centroid1 = B2Math.TransformPoint(xf1, fastShape.LocalCentroid);
                context.Centroid2 = B2Math.TransformPoint(xf2, fastShape.LocalCentroid);

                AABB box1 = fastShape.AABB;
                AABB box2 = fastShape.ComputeShapeAABB(xf2);
                AABB box = B2Math.AABB_Union(box1, box2);

                // Store this for later
                fastShape.AABB = box2;

                // No continuous collision for sensors
                if (fastShape.IsSensor)
                {
                    continue;
                }

                staticTree.Query(box, Core.DefaultMaskBits, _continuousQueryCallback, context);

                if (isBullet)
                {
                    kinematicTree.Query(box, Core.DefaultMaskBits, _continuousQueryCallback, context);
                    dynamicTree.Query(box, Core.DefaultMaskBits, _continuousQueryCallback, context);
                }
            }

            float speculativeDistance = Core.SpeculativeDistance;
            float aabbMargin = Core.b2_aabbMargin;

            if (context.Fraction < 1.0f)
            {
                // Handle time of impact event
                Rot q = B2Math.NLerp(sweep.Q1, sweep.Q2, context.Fraction);
                Vec2 c = B2Math.Lerp(sweep.C1, sweep.C2, context.Fraction);
                Vec2 origin = B2Math.Sub(c, B2Math.RotateVector(q, sweep.LocalCenter));

                // Advance body
                Transform transform = (origin, q);
                fastBodySim.Transform = transform;
                fastBodySim.Center = c;
                fastBodySim.Rotation0 = q;
                fastBodySim.Center0 = c;

                // Prepare AABBs for broad-phase
                shapeId = fastBody.HeadShapeId;
                while (shapeId != Core.NullIndex)
                {
                    Shape shape = shapes[shapeId];

                    // Must recompute aabb at the interpolated transform
                    AABB aabb = shape.ComputeShapeAABB(transform);
                    aabb.LowerBound.X -= speculativeDistance;
                    aabb.LowerBound.Y -= speculativeDistance;
                    aabb.UpperBound.X += speculativeDistance;
                    aabb.UpperBound.Y += speculativeDistance;
                    shape.AABB = aabb;

                    if (B2Math.AABB_Contains(shape.FatAABB, aabb) == false)
                    {
                        AABB fatAABB;
                        fatAABB.LowerBound.X = aabb.LowerBound.X - aabbMargin;
                        fatAABB.LowerBound.Y = aabb.LowerBound.Y - aabbMargin;
                        fatAABB.UpperBound.X = aabb.UpperBound.X + aabbMargin;
                        fatAABB.UpperBound.Y = aabb.UpperBound.Y + aabbMargin;
                        shape.FatAABB = fatAABB;

                        shape.EnlargedAABB = true;
                        fastBodySim.EnlargeAABB = true;
                    }

                    shapeId = shape.NextShapeId;
                }
            }
            else
            {
                // No time of impact event

                // Advance body
                fastBodySim.Rotation0 = fastBodySim.Transform.Q;
                fastBodySim.Center0 = fastBodySim.Center;

                // Prepare AABBs for broad-phase
                shapeId = fastBody.HeadShapeId;
                while (shapeId != Core.NullIndex)
                {
                    Shape shape = shapes[shapeId];

                    // shape.aabb is still valid

                    if (B2Math.AABB_Contains(shape.FatAABB, shape.AABB) == false)
                    {
                        AABB fatAABB;
                        fatAABB.LowerBound.X = shape.AABB.LowerBound.X - aabbMargin;
                        fatAABB.LowerBound.Y = shape.AABB.LowerBound.Y - aabbMargin;
                        fatAABB.UpperBound.X = shape.AABB.UpperBound.X + aabbMargin;
                        fatAABB.UpperBound.Y = shape.AABB.UpperBound.Y + aabbMargin;
                        shape.FatAABB = fatAABB;

                        shape.EnlargedAABB = true;
                        fastBodySim.EnlargeAABB = true;
                    }

                    shapeId = shape.NextShapeId;
                }
            }

            B2ObjectPool<ContinuousContext>.Shared.Return(context);
        }

        public static void FastBodyTask(int startIndex, int endIndex, int threadIndex, object taskContext)
        {
            StepContext stepContext = (StepContext)taskContext;

            Debug.Assert(startIndex <= endIndex);

            for (int i = startIndex; i < endIndex; ++i)
            {
                int simIndex = stepContext.FastBodies[i];
                SolveContinuous(stepContext.World, simIndex);
            }
        }

        public static void BulletBodyTask(int startIndex, int endIndex, int threadIndex, object taskContext)
        {
            StepContext stepContext = (StepContext)taskContext;

            Debug.Assert(startIndex <= endIndex);

            for (int i = startIndex; i < endIndex; ++i)
            {
                int simIndex = stepContext.BulletBodies[i];
                SolveContinuous(stepContext.World, simIndex);
            }
        }

        /// <summary>
        /// Solve with graph coloring
        /// 使用图形着色求解，求解器入库
        /// </summary>
        /// <param name="world"></param>
        /// <param name="stepContext"></param>
        public static void Solve(World world, StepContext stepContext)
        {
            var t1 = Stopwatch.GetTimestamp();

            world.StepIndex += 1;

            // 合并活跃岛屿
            Island.MergeAwakeIslands(world);

            var e = StopwatchHelper.GetElapsedTime(t1);
            world.Profile.BuildIslands = (float)e.TotalMilliseconds;

            SolverSet awakeSet = world.SolverSetArray[SolverSetType.AwakeSet];
            int awakeBodyCount = awakeSet.Sims.Count;
            if (awakeBodyCount == 0)
            {
                // Nothing to simulate, however I must still finish the broad-phase rebuild.
                // 没有需要物理模拟的活跃刚体，但仍然执行粗检测重建
                if (world.UserTreeTask != null)
                {
                    world.UserTreeTask.Wait();
                    world.UserTreeTask = null;
                    world.ActiveTaskCount -= 1;
                }

                world.BroadPhase.ValidateNoEnlarged();
                return;
            }

            // Prepare buffers for continuous collision (fast bodies)
            // 准备要计算连续碰撞的高速刚体和子弹刚体的缓存
            stepContext.FastBodyCount = 0;
            stepContext.FastBodies = B2ArrayPool<int>.Shared.Rent(awakeBodyCount); // b2AllocateStackItem(&world.stackAllocator, awakeBodyCount * sizeof(int), "fast bodies");
            stepContext.BulletBodyCount = 0;
            stepContext.BulletBodies = B2ArrayPool<int>.Shared.Rent(awakeBodyCount); // b2AllocateStackItem(&world.stackAllocator, awakeBodyCount * sizeof(int), "bullet bodies");

            // Solve constraints using graph coloring
            // 使用图着色解算约束
            {
                ConstraintGraph graph = world.ConstraintGraph;
                Span<GraphColor> colors = graph.Colors;

                stepContext.Sims = awakeSet.Sims;
                stepContext.States = awakeSet.States;

                // count contacts, joints, and colors
                // 统计接触点、关节和活跃色，有接触点或关节的着色图记为活跃
                int awakeContactCount = 0;
                int awakeJointCount = 0;
                int activeColorCount = 0;
                for (int i = 0; i < Core.GraphColorCount - 1; ++i)
                {
                    // 每种颜色的接触点数量、关节数量
                    int perColorContactCount = colors[i].Contacts.Count;
                    int perColorJointCount = colors[i].Joints.Count;
                    int occupancyCount = perColorContactCount + perColorJointCount;
                    activeColorCount += occupancyCount > 0 ? 1 : 0;
                    awakeContactCount += perColorContactCount;
                    awakeJointCount += perColorJointCount;
                }

                // 刚体移动事件数组，扩容到活跃物体数量
                world.BodyMoveEventArray.Resize(awakeBodyCount);

                // Each worker receives at most M blocks of work. The workers may receive less than there is not sufficient work.
                // Each block of work has a minimum number of elements (block size). This in turn may limit number of blocks.
                // If there are many elements then the block size is increased so there are still at most M blocks of work per worker.
                // M is a tunable number that has two goals:
                // 1. keep M small to reduce overhead
                // 2. keep M large enough for other workers to be able to steal work
                // The block size is a power of two to make math efficient.
                // 每个工人最多可获得M个工作区块。如果没有足够的工作，工人收到的区块可能会更少。
                // 每个区块都有一个最小元素数（区块大小）。这反过来又会限制区块的数量。
                // 如果有很多元素，那么区块的大小就会增加，这样每个工人最多只能得到M个区块。
                // M 是一个可调整的数字，有两个目标：
                // 1. 保持 M 较小，以减少开销
                // 2. 保持 M 足够大，以便其他工作者能够窃取工作。
                // 区块大小是 2 的幂次，以提高数学效率。

                int workerCount = world.WorkerCount;               // 工人数量
                int blocksPerWorker = 4;                           // 每个工人处理的区块数量
                int maxBlockCount = blocksPerWorker * workerCount; // 最大区块数量

                // Configure blocks for tasks that parallel-for bodies
                // 为并行刚体任务配置区块
                int bodyBlockSize = 1 << 5; // 默认区块大小，最小32
                int bodyBlockCount;         // 区块数量

                if (awakeBodyCount > bodyBlockSize * maxBlockCount)
                {
                    // Too many blocks, increase block size
                    // 如果：活跃刚体数量 > 区块大小 x 最大区块数量，则在保持区块数量为最大的情况下重新计算单个区块大小
                    bodyBlockSize = awakeBodyCount / maxBlockCount;
                    bodyBlockCount = maxBlockCount;
                }
                else
                {
                    // 活跃刚体数 < 区块大小 x 最大区块数量，默认区块大小是32，使用位运算得到区块数量
                    bodyBlockCount = ((awakeBodyCount - 1) >> 5) + 1;
                }

                // Configure blocks for tasks parallel-for each active graph color
                // The blocks are a mix of SIMD contact blocks and joint blocks
                // 为每个活跃图配置并行任务
                // 区块中包含SIMD化的接触点区块，以及关节区块
                Span<int> activeColorIndices = stackalloc int[Core.GraphColorCount]; // 活跃颜色数组，内容是活跃色的索引0~11

                Span<int> colorContactCounts = stackalloc int[Core.GraphColorCount];      // 活跃色的接触点经过SIMD合并后的数量
                Span<int> colorContactBlockSizes = stackalloc int[Core.GraphColorCount];  // 活跃色接触点任务区块大小
                Span<int> colorContactBlockCounts = stackalloc int[Core.GraphColorCount]; // 活跃色接触点任务区块数量

                Span<int> colorJointCounts = stackalloc int[Core.GraphColorCount];      // 活跃色的关节数量，不使用SIMD
                Span<int> colorJointBlockSizes = stackalloc int[Core.GraphColorCount];  // 活跃色关节任务区块大小
                Span<int> colorJointBlockCounts = stackalloc int[Core.GraphColorCount]; // 活跃色关节任务区块数量

                int graphBlockCount = 0; // 所有活跃色的区块总数，接触点区块数+关节区块数

                // c is the active color index
                int simdContactCount = 0; // 所有活跃色的接触点SIMD分批数
                int c = 0;                // 活跃颜色计数索引

                for (int i = 0; i < Core.GraphColorCount - 1; ++i)
                {
                    int colorContactCount = colors[i].Contacts.Count;
                    int colorJointCount = colors[i].Joints.Count;

                    // 遍历每个颜色，有接触点或关节的颜色，记为活跃色
                    if (colorContactCount + colorJointCount > 0)
                    {
                        activeColorIndices[c] = i;

                        // 4/8-way SIMD 根据CPU选择4路/8路SIMD，计算接触点SIMD批数，再加上SIMD分批后剩余数据的一个批次
                        int colorContactCountSIMD = colorContactCount > 0 ? ((colorContactCount - 1) >> Core.SimdShift) + 1 : 0;

                        colorContactCounts[c] = colorContactCountSIMD;

                        // determine the number of contact work blocks for this color
                        // 根据SIMD批数确定任务区块大小和数量
                        if (colorContactCountSIMD > blocksPerWorker * maxBlockCount)
                        {
                            // too many contact blocks
                            colorContactBlockSizes[c] = colorContactCountSIMD / maxBlockCount;
                            colorContactBlockCounts[c] = maxBlockCount;
                        }
                        else if (colorContactCountSIMD > 0)
                        {
                            // dividing by blocksPerWorker (4)
                            colorContactBlockSizes[c] = blocksPerWorker;
                            colorContactBlockCounts[c] = ((colorContactCountSIMD - 1) >> 2) + 1;
                        }
                        else
                        {
                            // no contacts in this color
                            colorContactBlockSizes[c] = 0;
                            colorContactBlockCounts[c] = 0;
                        }

                        colorJointCounts[c] = colorJointCount;

                        // determine number of joint work blocks for this color
                        // 根据接触点数量确定任务区块大小和数量
                        if (colorJointCount > blocksPerWorker * maxBlockCount)
                        {
                            // too many joint blocks
                            colorJointBlockSizes[c] = colorJointCount / maxBlockCount;
                            colorJointBlockCounts[c] = maxBlockCount;
                        }
                        else if (colorJointCount > 0)
                        {
                            // dividing by blocksPerWorker (4)
                            colorJointBlockSizes[c] = blocksPerWorker;
                            colorJointBlockCounts[c] = ((colorJointCount - 1) >> 2) + 1;
                        }
                        else
                        {
                            colorJointBlockSizes[c] = 0;
                            colorJointBlockCounts[c] = 0;
                        }

                        graphBlockCount += colorContactBlockCounts[c] + colorJointBlockCounts[c];
                        simdContactCount += colorContactCountSIMD;
                        c += 1;
                    }
                }

                activeColorCount = c;

                // Gather contact pointers for easy parallel-for traversal. Some may be null due to SIMD remainders.
                // 接触点仿真数量 = SIMD批数 x SIMD宽度
                int contactSimLength = Core.SimdWidth * simdContactCount;

                // 使用SIMD计算的全部接触点仿真数据
                // todo Pool
                ContactSim[] contacts = B2ArrayPool<ContactSim>.Shared.Rent(contactSimLength);

                // Gather joint pointers for easy parallel-for traversal.
                // 全部活跃关节仿真数据
                JointSim[] joints = B2ArrayPool<JointSim>.Shared.Rent(awakeJointCount);

                //int simdConstraintSize = b2GetContactConstraintSIMDByteCount();
                // 接触点仿真数据SIMD分批数组
                // todo Pool
                ContactConstraintSIMD[] simdContactConstraints = B2ArrayPool<ContactConstraintSIMD>.Shared.Rent(simdContactCount);
                ArrayHelper.FillFromPool(simdContactConstraints, 0, simdContactCount);

                // SIMD分批后剩余的接触点
                int overflowContactCount = colors[Core.OverflowIndex].Contacts.Count;
                ContactConstraint[] overflowContactConstraints = B2ArrayPool<ContactConstraint>.Shared.Rent(overflowContactCount);
                ArrayHelper.FillFromPool(overflowContactConstraints, 0, overflowContactCount);

                graph.Colors[Core.OverflowIndex].OverflowConstraints = overflowContactConstraints;

                // Distribute transient constraints to each graph color and build flat arrays of contact and joint pointers
                // 将瞬态约束分布到每个着色图中，并建立接触和关节的扁平数组
                {
                    int contactBase = 0; // 全量接触点仿真数据中SIMD批次的起始索引
                    int jointBase = 0;   // 全量关节仿真数据中的起始索引
                    for (int i = 0; i < activeColorCount; ++i)
                    {
                        int j = activeColorIndices[i]; // 活跃着色图的索引
                        GraphColor color = colors[j];  // 取得着色图

                        int colorContactCount = color.Contacts.Count;

                        if (colorContactCount == 0)
                        {
                            color.SimdConstraints = null;
                        }
                        else
                        {
                            // 从全量SIMD接触约束中截取
                            color.SimdConstraints = simdContactConstraints.AsMemory(contactBase);

                            var baseIndex = Core.SimdWidth * contactBase;
                            for (int k = 0; k < colorContactCount; ++k)
                            {
                                contacts[baseIndex + k] = color.Contacts[k];
                            }

                            // remainder
                            // 单个着色图中的SIMD批数
                            int colorContactCountSIMD = ((colorContactCount - 1) >> Core.SimdShift) + 1;

                            // 遍历重置SIMD所有元素位置
                            var start = baseIndex + colorContactCount;
                            var end = baseIndex + Core.SimdWidth * colorContactCountSIMD;
                            contacts.AsSpan().Slice(start, end - start).Clear();

                            // for (int k = colorContactCount; k < Core.SimdWidth * colorContactCountSIMD; ++k)
                            // {
                            //     contacts[baseIndex + k] = null!;
                            // }

                            contactBase += colorContactCountSIMD;
                        }

                        int colorJointCount = color.Joints.Count;
                        for (int k = 0; k < colorJointCount; ++k)
                        {
                            joints[jointBase + k] = color.Joints.Data[k];
                        }

                        jointBase += colorJointCount;
                    }

                    Debug.Assert(contactBase == simdContactCount);
                    Debug.Assert(jointBase == awakeJointCount);
                }

                // Define work blocks for preparing contacts and storing contact impulses
                // 为准备接触点和存储接点冲量定义工作区块
                int contactBlockSize = blocksPerWorker;
                int contactBlockCount = simdContactCount > 0 ? ((simdContactCount - 1) >> 2) + 1 : 0;
                if (simdContactCount > contactBlockSize * maxBlockCount)
                {
                    // Too many blocks, increase block size
                    contactBlockSize = simdContactCount / maxBlockCount;
                    contactBlockCount = maxBlockCount;
                }

                // Define work blocks for preparing joints
                int jointBlockSize = blocksPerWorker;
                int jointBlockCount = awakeJointCount > 0 ? ((awakeJointCount - 1) >> 2) + 1 : 0;
                if (awakeJointCount > jointBlockSize * maxBlockCount)
                {
                    // Too many blocks, increase block size
                    jointBlockSize = awakeJointCount / maxBlockCount;
                    jointBlockCount = maxBlockCount;
                }

                // 计算步骤数量
                int stageCount = 0;

                // b2_stagePrepareJoints
                stageCount += 1;

                // b2_stagePrepareContacts
                stageCount += 1;

                // b2_stageIntegrateVelocities
                stageCount += 1;

                // b2_stageWarmStart
                stageCount += activeColorCount;

                // b2_stageSolve
                stageCount += activeColorCount;

                // b2_stageIntegratePositions
                stageCount += 1;

                // b2_stageRelax
                stageCount += activeColorCount;

                // b2_stageRestitution
                stageCount += activeColorCount;

                // b2_stageStoreImpulses
                stageCount += 1;

                SolverStage[] stages = B2ArrayPool<SolverStage>.Shared.Rent(stageCount);
                ArrayHelper.FillFromPool(stages, 0, stageCount);

                SolverBlock[] bodyBlocks = B2ArrayPool<SolverBlock>.Shared.Rent(bodyBlockCount);
                ArrayHelper.FillFromPool(bodyBlocks, 0, bodyBlockCount);

                SolverBlock[] contactBlocks = B2ArrayPool<SolverBlock>.Shared.Rent(contactBlockCount);
                ArrayHelper.FillFromPool(contactBlocks, 0, contactBlockCount);

                SolverBlock[] jointBlocks = B2ArrayPool<SolverBlock>.Shared.Rent(jointBlockCount);
                ArrayHelper.FillFromPool(jointBlocks, 0, jointBlockCount);

                SolverBlock[] graphBlocks = B2ArrayPool<SolverBlock>.Shared.Rent(graphBlockCount);
                ArrayHelper.FillFromPool(graphBlocks, 0, graphBlockCount);

                // Split an awake island. This modifies:
                // - stack allocator
                // - world island array and solver set
                // - island indices on bodies, contacts, and joints
                // I'm squeezing this task in here because it may be expensive and this is a safe place to put it.
                // Note: cannot split islands in parallel with FinalizeBodies
                // 分割一个活跃的岛屿。这将修改
                // - 堆栈分配器
                // - 世界岛数组和求解器集
                // - 刚体、接触点和关节上的岛屿索引
                // 我把这个任务放在这里是因为它可能很昂贵，而且放在这里比较安全。
                // 注意：不能与在并行执行`FinalizeBodies`时分割岛屿
                if (world.SplitIslandId != Core.NullIndex)
                {
                    world.TaskCount += 1;
                    Island.SplitIslandTask(world);
                }

                // Prepare body work blocks
                for (int i = 0; i < bodyBlockCount; ++i)
                {
                    SolverBlock block = bodyBlocks[i];
                    block.StartIndex = i * bodyBlockSize;
                    block.Count = (short)bodyBlockSize;
                    block.BlockType = (short)SolverBlockType.BodyBlock;
                    block.SyncIndex = 0;
                }

                bodyBlocks[bodyBlockCount - 1].Count = (short)(awakeBodyCount - (bodyBlockCount - 1) * bodyBlockSize);

                // Prepare joint work blocks
                for (int i = 0; i < jointBlockCount; ++i)
                {
                    SolverBlock block = jointBlocks[i];
                    block.StartIndex = i * jointBlockSize;
                    block.Count = (short)jointBlockSize;
                    block.BlockType = (short)SolverBlockType.JointBlock;
                    block.SyncIndex = 0;
                }

                if (jointBlockCount > 0)
                {
                    jointBlocks[jointBlockCount - 1].Count = (short)(awakeJointCount - (jointBlockCount - 1) * jointBlockSize);
                }

                // Prepare contact work blocks
                for (int i = 0; i < contactBlockCount; ++i)
                {
                    SolverBlock block = contactBlocks[i];
                    block.StartIndex = i * contactBlockSize;
                    block.Count = (short)contactBlockSize;
                    block.BlockType = (short)SolverBlockType.ContactBlock;
                    block.SyncIndex = 0;
                }

                if (contactBlockCount > 0)
                {
                    contactBlocks[contactBlockCount - 1].Count =
                        (short)(simdContactCount - (contactBlockCount - 1) * contactBlockSize);
                }

                // Prepare graph work blocks
                SolverBlock[][] graphColorBlocks = new SolverBlock[Core.GraphColorCount][];
                var baseGraphBlock = graphBlocks;
                var baseBlockIndex = 0;
                for (int i = 0; i < activeColorCount; ++i)
                {
                    var start = baseBlockIndex;

                    int colorJointBlockCount = colorJointBlockCounts[i];
                    int colorJointBlockSize = colorJointBlockSizes[i];
                    for (int j = 0; j < colorJointBlockCount; ++j)
                    {
                        SolverBlock block = baseGraphBlock[baseBlockIndex + j];
                        block.StartIndex = j * colorJointBlockSize;
                        block.Count = (short)colorJointBlockSize;
                        block.BlockType = (short)SolverBlockType.GraphJointBlock;
                        block.SyncIndex = 0;
                    }

                    if (colorJointBlockCount > 0)
                    {
                        baseGraphBlock[baseBlockIndex + colorJointBlockCount - 1].Count =
                            (short)(colorJointCounts[i] - (colorJointBlockCount - 1) * colorJointBlockSize);
                        baseBlockIndex += colorJointBlockCount;
                    }

                    int colorContactBlockCount = colorContactBlockCounts[i];
                    int colorContactBlockSize = colorContactBlockSizes[i];
                    for (int j = 0; j < colorContactBlockCount; ++j)
                    {
                        SolverBlock block = baseGraphBlock[baseBlockIndex + j];
                        block.StartIndex = j * colorContactBlockSize;
                        block.Count = (short)colorContactBlockSize;
                        block.BlockType = (short)SolverBlockType.GraphContactBlock;
                        block.SyncIndex = 0;
                    }

                    if (colorContactBlockCount > 0)
                    {
                        baseGraphBlock[baseBlockIndex + colorContactBlockCount - 1].Count =
                            (short)(colorContactCounts[i] - (colorContactBlockCount - 1) * colorContactBlockSize);
                        baseBlockIndex += colorContactBlockCount;
                    }

                    var end = baseBlockIndex;

                    graphColorBlocks[i] = baseGraphBlock.AsSpan(start, end - start).ToArray();
                }

                Debug.Assert(graphBlockCount == baseBlockIndex);

                var stageIndex = 0;
                var stage = stages[stageIndex];

                // Prepare joints
                stage.Type = SolverStageType.PrepareJoints;
                stage.Blocks = jointBlocks;
                stage.BlockCount = jointBlockCount;
                stage.ColorIndex = -1;
                stage.CompletionCount = 0;

                // Prepare contacts
                stageIndex++;
                stage = stages[stageIndex];
                stage.Type = SolverStageType.PrepareContacts;
                stage.Blocks = contactBlocks;
                stage.BlockCount = contactBlockCount;
                stage.ColorIndex = -1;
                stage.CompletionCount = 0;

                // Integrate velocities
                stageIndex++;
                stage = stages[stageIndex];
                stage.Type = SolverStageType.IntegrateVelocities;
                stage.Blocks = bodyBlocks;
                stage.BlockCount = bodyBlockCount;
                stage.ColorIndex = -1;
                stage.CompletionCount = 0;

                // Warm start
                for (int i = 0; i < activeColorCount; ++i)
                {
                    stageIndex++;
                    stage = stages[stageIndex];
                    stage.Type = SolverStageType.WarmStart;
                    stage.BlockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
                    stage.Blocks = graphColorBlocks[i];
                    stage.ColorIndex = activeColorIndices[i];
                    stage.CompletionCount = 0;
                }

                // Solve graph
                for (int i = 0; i < activeColorCount; ++i)
                {
                    stageIndex++;
                    stage = stages[stageIndex];
                    stage.Type = SolverStageType.Solve;
                    stage.BlockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
                    stage.Blocks = graphColorBlocks[i];
                    stage.ColorIndex = activeColorIndices[i];
                    stage.CompletionCount = 0;
                }

                // Integrate positions
                stageIndex++;
                stage = stages[stageIndex];
                stage.Type = SolverStageType.IntegratePositions;
                stage.Blocks = bodyBlocks;
                stage.BlockCount = bodyBlockCount;
                stage.ColorIndex = -1;
                stage.CompletionCount = 0;

                // Relax constraints
                // 放宽约束
                for (int i = 0; i < activeColorCount; ++i)
                {
                    stageIndex++;
                    stage = stages[stageIndex];
                    stage.Type = SolverStageType.Relax;
                    stage.BlockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
                    stage.Blocks = graphColorBlocks[i];
                    stage.ColorIndex = activeColorIndices[i];
                    stage.CompletionCount = 0;
                }

                // Restitution
                // Note: joint blocks mixed in, could have joint limit restitution
                // 弹性计算，包括接触点区块和关节区块，因为有关节弹性限制
                for (int i = 0; i < activeColorCount; ++i)
                {
                    stageIndex++;
                    stage = stages[stageIndex];
                    stage.Type = SolverStageType.Restitution;
                    stage.BlockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
                    stage.Blocks = graphColorBlocks[i];
                    stage.ColorIndex = activeColorIndices[i];
                    stage.CompletionCount = 0;
                }

                // Store impulses
                stageIndex++;
                stage = stages[stageIndex];
                stage.Type = SolverStageType.StoreImpulses;
                stage.Blocks = contactBlocks;
                stage.BlockCount = contactBlockCount;
                stage.ColorIndex = -1;
                stage.CompletionCount = 0;
                stageIndex++;

                Debug.Assert(stageIndex == stageCount);

                Debug.Assert(workerCount <= Core.MaxWorkers);

                stepContext.Graph = graph;
                stepContext.Joints = joints;
                stepContext.Contacts = contacts;
                stepContext.SimdContactConstraints = simdContactConstraints;
                stepContext.ActiveColorCount = activeColorCount;
                stepContext.WorkerCount = workerCount;
                stepContext.StageCount = stageCount;
                stepContext.Stages = stages;
                stepContext.AtomicSyncBits = 0;

                world.Profile.PrepareTasks = (float)StopwatchHelper.GetElapsedTime(t1).TotalMilliseconds;

                // Must use worker index because thread 0 can be assigned multiple tasks by enkiTS

                #region SolverTasks

                t1 = Stopwatch.GetTimestamp();

                world.SplitIslandId = Core.NullIndex;

                // Finish constraint solve

                for (int i = 0; i < workerCount; ++i)
                {
                    var ctx = new WorkerContext
                    {
                        Context = stepContext,
                        WorkerIndex = i
                    };
                    SolverTask(ctx);
                }

                world.TaskCount += workerCount;
                world.Profile.SolverTasks = (float)StopwatchHelper.GetElapsedTime(t1).TotalMilliseconds;

                #endregion SolverTasks

                t1 = Stopwatch.GetTimestamp();

                // Prepare contact, enlarged body, and island bit sets used in body finalization.
                int awakeIslandCount = awakeSet.Islands.Count;
                for (int i = 0; i < world.WorkerCount; ++i)
                {
                    TaskContext taskContext = world.TaskContextArray[i];
                    taskContext.EnlargedSimBitSet.SetBitCountAndClear(awakeBodyCount);
                    taskContext.AwakeIslandBitSet.SetBitCountAndClear(awakeIslandCount);
                    taskContext.SplitIslandId = Core.NullIndex;
                    taskContext.SplitSleepTime = 0.0f;
                }

                // Finalize bodies. Must happen after the constraint solver and after island splitting.
                var finalizeBodiesTask = world.EnqueueTaskFcn(FinalizeBodiesTask, awakeBodyCount, 64, stepContext, world.UserTaskContext);
                world.TaskCount += 1;
                if (finalizeBodiesTask != null!)
                {
                    world.FinishTaskFcn(finalizeBodiesTask, world.UserTaskContext);
                }

                world.Profile.FinalizeBodies = (float)StopwatchHelper.GetElapsedTime(t1).TotalMilliseconds;
                t1 = Stopwatch.GetTimestamp();

                ArrayHelper.ReturnToPool(graphBlocks, 0, graphBlockCount);
                ArrayHelper.ReturnToPool(jointBlocks, 0, jointBlockCount);
                ArrayHelper.ReturnToPool(contactBlocks, 0, contactBlockCount);
                ArrayHelper.ReturnToPool(bodyBlocks, 0, bodyBlockCount);
                ArrayHelper.ReturnToPool(stages, 0, stageCount);
                ArrayHelper.ReturnToPool(simdContactConstraints, 0, simdContactCount);
                ArrayHelper.ReturnToPool(overflowContactConstraints, 0, overflowContactCount);

                B2ArrayPool<SolverBlock>.Shared.Return(graphBlocks);
                B2ArrayPool<SolverBlock>.Shared.Return(jointBlocks);
                B2ArrayPool<SolverBlock>.Shared.Return(contactBlocks);
                B2ArrayPool<SolverBlock>.Shared.Return(bodyBlocks);
                B2ArrayPool<SolverStage>.Shared.Return(stages);

                B2ArrayPool<ContactConstraint>.Shared.Return(overflowContactConstraints);
                B2ArrayPool<ContactConstraintSIMD>.Shared.Return(simdContactConstraints);

                B2ArrayPool<ContactSim>.Shared.Return(contacts);
                B2ArrayPool<JointSim>.Shared.Return(joints);
            }

            world.Profile.SolveConstraints = (float)StopwatchHelper.GetElapsedTime(t1).TotalMilliseconds;
            t1 = Stopwatch.GetTimestamp();

            // Report hit events
            // todo perhaps optimize this with a bitset
            {
                Debug.Assert((world.ContactHitArray).Count == 0);

                float threshold = world.HitEventThreshold;
                var colors = world.ConstraintGraph.Colors;
                for (int i = 0; i < Core.GraphColorCount; ++i)
                {
                    GraphColor color = colors[i];
                    int contactCount = color.Contacts.Count;
                    var contactSims = color.Contacts.Data;
                    for (int j = 0; j < contactCount; ++j)
                    {
                        ContactSim contactSim = contactSims[j];
                        if (!contactSim.SimFlags.IsSet(ContactSimFlags.EnableHitEvent))
                        {
                            continue;
                        }

                        ContactHitEvent evt = new();
                        evt.ApproachSpeed = threshold;

                        bool hit = false;
                        int pointCount = contactSim.Manifold.PointCount;
                        for (int k = 0; k < pointCount; ++k)
                        {
                            ref readonly ManifoldPoint mp = ref contactSim.Manifold.Points[k];
                            float approachSpeed = -mp.NormalVelocity;
                            if (approachSpeed > evt.ApproachSpeed && mp.NormalImpulse > 0.0f)
                            {
                                evt.ApproachSpeed = approachSpeed;
                                evt.Point = mp.Point;
                                hit = true;
                            }
                        }

                        if (hit == true)
                        {
                            evt.Normal = contactSim.Manifold.Normal;

                            world.ShapeArray.CheckId(contactSim.ShapeIdA);
                            world.ShapeArray.CheckId(contactSim.ShapeIdB);
                            Shape shapeA = world.ShapeArray[contactSim.ShapeIdA];
                            Shape shapeB = world.ShapeArray[contactSim.ShapeIdB];
                            evt.ShapeIdA = new ShapeId(shapeA.Id + 1, world.WorldId, shapeA.Revision);
                            evt.ShapeIdB = new ShapeId(shapeB.Id + 1, world.WorldId, shapeB.Revision);

                            world.ContactHitArray.Push(evt);
                        }
                    }
                }
            }

            world.Profile.HitEvents = (float)StopwatchHelper.GetElapsedTime(t1).TotalMilliseconds;
            t1 = Stopwatch.GetTimestamp();

            // Finish the user tree task that was queued earlier in the time step. This must be complete before touching the broad-phase.
            if (world.UserTreeTask != null)
            {
                world.UserTreeTask.Wait();
                world.UserTreeTask = null;
                world.ActiveTaskCount -= 1;
            }

            world.BroadPhase.ValidateNoEnlarged();

            // Gather bits for all sim bodies that have enlarged AABBs
            var simBitSet = world.TaskContextArray[0].EnlargedSimBitSet;
            for (int i = 1; i < world.WorkerCount; ++i)
            {
                simBitSet.InPlaceUnion(world.TaskContextArray[i].EnlargedSimBitSet);
            }

            // Enlarge broad-phase proxies and build move array
            // Apply shape AABB changes to broad-phase. This also create the move array which must be
            // in deterministic order. I'm tracking sim bodies because the number of shape ids can be huge.
            {
                BroadPhase broadPhase = world.BroadPhase;
                var shapes = world.ShapeArray;
                var wordCount = simBitSet.BlockCount;
                var bits = simBitSet.Bits;
                for (uint k = 0; k < wordCount; ++k)
                {
                    var word = bits[k];
                    while (word != 0)
                    {
                        int ctz = BitTool.CTZ64(word);
                        var bodySimIndex = 64 * k + ctz;

                        // cache misses
                        Debug.Assert(bodySimIndex < awakeSet.Sims.Count);
                        BodySim bodySim = awakeSet.Sims.Data[bodySimIndex];

                        world.BodyArray.CheckIndex(bodySim.BodyId);
                        Body body = world.BodyArray[bodySim.BodyId];

                        int shapeId = body.HeadShapeId;
                        while (shapeId != Core.NullIndex)
                        {
                            shapes.CheckId(shapeId);
                            Shape shape = shapes[shapeId];

                            if (shape.EnlargedAABB)
                            {
                                Debug.Assert(shape.IsFast == false);

                                broadPhase.EnlargeProxy(shape.ProxyKey, shape.FatAABB);
                                shape.EnlargedAABB = false;
                            }
                            else if (shape.IsFast)
                            {
                                // Shape is fast. It's aabb will be enlarged in continuous collision.
                                broadPhase.BufferMove(shape.ProxyKey);
                            }

                            shapeId = shape.NextShapeId;
                        }

                        // Clear the smallest set bit
                        word = word & (word - 1);
                    }
                }
            }

            world.BroadPhase.ValidateBroadphase();

            world.Profile.Broadphase = (float)StopwatchHelper.GetElapsedTime(t1).TotalMilliseconds;
            t1 = Stopwatch.GetTimestamp();

            // Parallel continuous collision
            if (stepContext.FastBodyCount > 0)
            {
                // fast bodies
                int minRange = 8;
                var userFastBodyTask =
                    world.EnqueueTaskFcn(FastBodyTask, stepContext.FastBodyCount, minRange, stepContext, world.UserTaskContext);
                world.TaskCount += 1;
                if (userFastBodyTask != null)
                {
                    world.FinishTaskFcn(userFastBodyTask, world.UserTaskContext);
                }
            }

            // Serially enlarge broad-phase proxies for fast shapes
            // Doing this here so that bullet shapes see them
            {
                BroadPhase broadPhase = world.BroadPhase;
                DynamicTree dynamicTree = broadPhase.Trees[(int)BodyType.DynamicBody];
                var bodies = world.BodyArray;
                var shapes = world.ShapeArray;

                var fastBodies = stepContext.FastBodies;
                int fastBodyCount = stepContext.FastBodyCount;

                // This loop has non-deterministic order but it shouldn't affect the result
                for (int i = 0; i < fastBodyCount; ++i)
                {
                    Debug.Assert(0 <= fastBodies[i] && fastBodies[i] < awakeSet.Sims.Count);
                    BodySim fastBodySim = awakeSet.Sims.Data[fastBodies[i]];
                    if (fastBodySim.EnlargeAABB == false)
                    {
                        continue;
                    }

                    // clear flag
                    fastBodySim.EnlargeAABB = false;

                    bodies.CheckIndex(fastBodySim.BodyId);
                    var fastBody = bodies[fastBodySim.BodyId];

                    int shapeId = fastBody.HeadShapeId;
                    while (shapeId != Core.NullIndex)
                    {
                        Shape shape = shapes[shapeId];
                        if (shape.EnlargedAABB == false)
                        {
                            shapeId = shape.NextShapeId;
                            continue;
                        }

                        // clear flag
                        shape.EnlargedAABB = false;

                        int proxyKey = shape.ProxyKey;
                        int proxyId = BroadPhase.ProxyId(proxyKey);
                        Debug.Assert(BroadPhase.ProxyType(proxyKey) == (int)BodyType.DynamicBody);

                        // all fast shapes should already be in the move buffer
                        Debug.Assert(broadPhase.MoveSet.ContainsKey(proxyKey + 1));

                        dynamicTree.EnlargeProxy(proxyId, shape.FatAABB);

                        shapeId = shape.NextShapeId;
                    }
                }
            }

            if (stepContext.BulletBodyCount > 0)
            {
                // bullet bodies
                int minRange = 8;
                var userBulletBodyTask = world.EnqueueTaskFcn(
                    BulletBodyTask,
                    stepContext.BulletBodyCount,
                    minRange,
                    stepContext,
                    world.UserTaskContext);
                world.TaskCount += 1;
                if (userBulletBodyTask != null)
                {
                    world.FinishTaskFcn(userBulletBodyTask, world.UserTaskContext);
                }
            }

            // Serially enlarge broad-phase proxies for bullet shapes
            {
                BroadPhase broadPhase = world.BroadPhase;
                DynamicTree dynamicTree = broadPhase.Trees[(int)BodyType.DynamicBody];
                var bodies = world.BodyArray;
                var shapes = world.ShapeArray;

                // Serially enlarge broad-phase proxies for bullet shapes
                var bulletBodies = stepContext.BulletBodies;
                int bulletBodyCount = stepContext.BulletBodyCount;

                // This loop has non-deterministic order but it shouldn't affect the result
                for (int i = 0; i < bulletBodyCount; ++i)
                {
                    Debug.Assert(0 <= bulletBodies[i] && bulletBodies[i] < awakeSet.Sims.Count);
                    BodySim bulletBodySim = awakeSet.Sims.Data[bulletBodies[i]];
                    if (bulletBodySim.EnlargeAABB == false)
                    {
                        continue;
                    }

                    // clear flag
                    bulletBodySim.EnlargeAABB = false;

                    bodies.CheckIndex(bulletBodySim.BodyId);
                    var bulletBody = bodies[bulletBodySim.BodyId];

                    int shapeId = bulletBody.HeadShapeId;
                    while (shapeId != Core.NullIndex)
                    {
                        Shape shape = shapes[shapeId];
                        if (shape.EnlargedAABB == false)
                        {
                            shapeId = shape.NextShapeId;
                            continue;
                        }

                        // clear flag
                        shape.EnlargedAABB = false;

                        int proxyKey = shape.ProxyKey;
                        int proxyId = BroadPhase.ProxyId(proxyKey);
                        Debug.Assert(BroadPhase.ProxyType(proxyKey) == (int)BodyType.DynamicBody);

                        // all fast shapes should already be in the move buffer
                        Debug.Assert(broadPhase.MoveSet.ContainsKey(proxyKey + 1));

                        dynamicTree.EnlargeProxy(proxyId, shape.FatAABB);

                        shapeId = shape.NextShapeId;
                    }
                }
            }

            //b2FreeStackItem(&world.stackAllocator, stepContext.bulletBodies);
            B2ArrayPool<int>.Shared.Return(stepContext.BulletBodies, true);
            stepContext.BulletBodies = null!;
            stepContext.BulletBodyCount = 0;

            //b2FreeStackItem(&world.stackAllocator, stepContext.fastBodies);
            B2ArrayPool<int>.Shared.Return(stepContext.FastBodies, true);
            stepContext.FastBodies = null!;
            stepContext.FastBodyCount = 0;

            world.Profile.Continuous = (float)StopwatchHelper.GetElapsedTime(t1).TotalMilliseconds;
            t1 = Stopwatch.GetTimestamp();

            // Island sleeping
            // This must be done last because putting islands to sleep invalidates the enlarged body bits.
            if (world.EnableSleep == true)
            {
                // Collect split island candidate for the next time step. No need to split if sleeping is disabled.
                Debug.Assert(world.SplitIslandId == Core.NullIndex);
                float splitSleepTimer = 0.0f;
                for (int i = 0; i < world.WorkerCount; ++i)
                {
                    TaskContext taskContext = world.TaskContextArray[i];
                    if (taskContext.SplitIslandId != Core.NullIndex && taskContext.SplitSleepTime > splitSleepTimer)
                    {
                        world.SplitIslandId = taskContext.SplitIslandId;
                        splitSleepTimer = taskContext.SplitSleepTime;
                    }
                }

                BitSet awakeIslandBitSet = world.TaskContextArray[0].AwakeIslandBitSet;
                for (int i = 1; i < world.WorkerCount; ++i)
                {
                    awakeIslandBitSet.InPlaceUnion(world.TaskContextArray[i].AwakeIslandBitSet);
                }

                // Need to process in reverse because this moves islands to sleeping solver sets.
                var islands = awakeSet.Islands.Data;
                int count = awakeSet.Islands.Count;
                for (int islandIndex = count - 1; islandIndex >= 0; islandIndex -= 1)
                {
                    if (awakeIslandBitSet.GetBit(islandIndex) == true)
                    {
                        // this island is still awake
                        continue;
                    }

                    IslandSim island = islands[islandIndex];
                    int islandId = island.IslandId;

                    SolverSet.TrySleepIsland(world, islandId);
                }

                SolverSet.ValidateSolverSets(world);
            }

            world.Profile.SleepIslands = (float)StopwatchHelper.GetElapsedTime(t1).TotalMilliseconds;
        }
    }
}