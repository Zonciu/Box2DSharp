using System;
using System.Collections.Concurrent;
using System.ComponentModel;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;

namespace Box2DSharp
{
    // Per thread task storage

    /// The world class manages all physics entities, dynamic simulation,
    /// and asynchronous queries. The world also contains efficient memory
    /// management facilities.
    public class World : IDisposable
    {
        #region Fields

        public static readonly World?[] Worlds = new World[Core.MaxWorlds];

        /// <summary>
        /// 粗检测
        /// </summary>
        public BroadPhase BroadPhase = null!;

        /// <summary>
        /// 约束着色图
        /// </summary>
        public ConstraintGraph ConstraintGraph = null!;

        // The body id pool is used to allocate and recycle body ids. Body ids
        // provide a stable identifier for users, but incur caches misses when used
        // to access body data. Aligns with b2Body.
        public IdPool BodyIdPool = null!;

        // This is a sparse array that maps body ids to the body data
        // stored in solver sets. As sims move within a set or across set.
        // Indices come from id pool.
        public B2Array<Body> BodyArray = null!;

        // Provides free list for solver sets.
        public IdPool SolverSetIdPool = null!;

        /// <summary>
        /// Solvers sets allow sims to be stored in contiguous arrays.
        /// <br/>[0] The first set is all static sims.
        /// <br/>[1] The second set is active sims.
        /// <br/>[2] The third set is disabled sims.
        /// <br/>[N] The remaining sets are sleeping islands.
        /// </summary>
        public B2Array<SolverSet> SolverSetArray = null!;

        // Used to create stable ids for joints
        public IdPool JointIdPool = null!;

        // This is a sparse array that maps joint ids to the joint data stored in the constraint graph
        // or in the solver sets.
        public B2Array<Joint> JointArray = null!;

        // Used to create stable ids for contacts
        public IdPool ContactIdPool = null!;

        // This is a sparse array that maps contact ids to the contact data stored in the constraint graph
        // or in the solver sets.
        public B2Array<Contact> ContactArray = null!;

        // Used to create stable ids for islands
        public IdPool IslandIdPool = null!;

        // This is a sparse array that maps island ids to the island data stored in the solver sets.
        public B2Array<Island> IslandArray = null!;

        public IdPool ShapeIdPool = null!;

        public IdPool ChainIdPool = null!;

        // These are sparse arrays that point into the pools above
        public B2Array<Shape> ShapeArray = null!;

        public B2Array<ChainShape> ChainArray = null!;

        /// <summary>
        /// Per thread storage
        /// 任务上下文列表，每个线程一个
        /// </summary>
        public B2Array<TaskContext> TaskContextArray = null!;

        /// <summary>
        /// 刚体移动事件
        /// </summary>
        public B2Array<BodyMoveEvent> BodyMoveEventArray = null!;

        /// <summary>
        /// 传感器触碰开始事件
        /// </summary>
        public B2Array<SensorBeginTouchEvent> SensorBeginEventArray = null!;

        /// <summary>
        /// 传感器触碰结束事件
        /// </summary>
        public B2Array<SensorEndTouchEvent> SensorEndEventArray = null!;

        /// <summary>
        /// 接触点触碰开始事件
        /// </summary>
        public B2Array<ContactBeginTouchEvent> ContactBeginArray = null!;

        /// <summary>
        /// 接触点触碰结束事件
        /// </summary>
        public B2Array<ContactEndTouchEvent> ContactEndArray = null!;

        /// <summary>
        /// 接触点碰撞事件
        /// </summary>
        public B2Array<ContactHitEvent> ContactHitArray = null!;

        /// <summary>
        /// Used to track debug draw
        /// 刚体调试绘制跟踪
        /// </summary>
        public BitSet DebugBodySet = null!;

        /// <summary>
        /// 关节调试绘制跟踪
        /// </summary>
        public BitSet DebugJointSet = null!;

        /// <summary>
        /// 接触点调试绘制跟踪
        /// </summary>
        public BitSet DebugContactSet = null!;

        /// <summary>
        /// Id that is incremented every time step
        /// 步进索引，每一步自增一次
        /// </summary>
        public ulong StepIndex;

        /// <summary>
        /// Identify islands for splitting as follows:
        /// - I want to split islands so smaller islands can sleep
        /// - when a body comes to rest and its sleep timer trips, I can look at the island and flag it for splitting
        ///   if it has removed constraints
        /// - islands that have removed constraints must be put split first because I don't want to wake bodies incorrectly
        /// - otherwise I can use the awake islands that have bodies wanting to sleep as the splitting candidates
        /// - if no bodies want to sleep then there is no reason to perform island splitting
        /// <br/>确定要分割的岛屿如下：
        /// <br/> - 我想拆分岛屿，让较小的岛屿可以休眠
        /// <br/> - 当一个刚体进入休眠状态，其休眠计时器启动时，如果它已移除约束，我可以查看该岛，并标记它以便分割
        /// <br/> - 因为我不想错误地唤醒刚体，所以必须先拆分已移除约束的岛屿
        /// <br/> - 否则，我可以使用有要休眠的刚体的活跃岛屿作为分割候选岛
        /// <br/> - 如果没有刚体要休眠，就不需要分割岛屿
        /// </summary>
        public int SplitIslandId;

        /// <summary>
        /// 世界重力
        /// </summary>
        public Vec2 Gravity;

        /// <summary>
        /// 碰撞接触事件阈值
        /// </summary>
        public float HitEventThreshold;

        public float RestitutionThreshold;

        public float MaxLinearVelocity;

        public float ContactPushOutVelocity;

        public float ContactHertz;

        public float ContactDampingRatio;

        public float JointHertz;

        public float JointDampingRatio;

        public ushort Revision;

        public Profile Profile = null!;

        public PreSolveFcn? PreSolveFcn;

        public object PreSolveContext = null!;

        public CustomFilterFcn? CustomFilterFcn;

        public object CustomFilterContext = null!;

        public int WorkerCount;

        public EnqueueTaskCallback EnqueueTaskFcn = null!;

        public FinishTaskCallback FinishTaskFcn = null!;

        public object UserTaskContext = null!;

        /// <summary>
        /// 粗检测任务
        /// </summary>
        public ManualResetEventSlim? UserTreeTask;

        /// <summary>
        /// Remember type step used for reporting forces and torques
        /// 子步时间分片数的倒数，从步进上下文计算并储存到世界中，用于计算受力和扭矩时
        /// </summary>
        public float InvH;

        /// <summary>
        /// 活跃任务数
        /// </summary>
        public int ActiveTaskCount;

        /// <summary>
        /// 任务数量
        /// </summary>
        public int TaskCount;

        /// <summary>
        /// 世界Id
        /// </summary>
        public ushort WorldId;

        /// <summary>
        /// 允许刚体休眠
        /// </summary>
        public bool EnableSleep;

        /// <summary>
        /// 锁定世界，Step执行时会锁定，锁定时不允许从外部修改世界
        /// </summary>
        public bool Locked;

        /// <summary>
        /// 启用热启动
        /// </summary>
        public bool EnableWarmStarting;

        /// <summary>
        /// 启用连续碰撞
        /// </summary>
        public bool EnableContinuous;

        public bool InUse;

        public WorkerStack WorkerStack;

        #endregion Fields

        #region Methods

        public void Dispose()
        { }

        public static World GetWorldFromId(WorldId id)
        {
            Debug.Assert(id.Index1 is >= 1 and <= Core.MaxWorlds);
            Worlds[id.Index1 - 1] ??= new();
            var world = Worlds[id.Index1 - 1]!;
            Debug.Assert(id.Index1 == world.WorldId + 1);
            Debug.Assert(id.Revision == world.Revision);
            return world;
        }

        public static World GetWorld(int index)
        {
            Debug.Assert(index is >= 0 and < Core.MaxWorlds);
            Worlds[index] ??= new();
            var world = Worlds[index]!;
            Debug.Assert(world.WorldId == index);
            return world;
        }

        public static World GetWorldLocked(int index)
        {
            Debug.Assert(index is >= 0 and < Core.MaxWorlds);
            var world = Worlds[index]!;
            Debug.Assert(world.WorldId == index);

            if (!world.Locked)
            {
                return world;
            }

            throw new Exception($"Worlds[{index}] should not be locked");
        }

        public static object DefaultAddTaskFcn(TaskCallback taskCallback, int count, int minRange, object taskContext, object userContext)
        {
            // 默认单线程
            taskCallback(0, count, 0, taskContext);
            return null!;
        }

        public static void DefaultFinishTaskFcn(object userTask, object userContext)
        {
            // Do nothing
        }

        public static WorldId CreateWorld(in WorldDef def)
        {
            Debug.Assert(Core.MaxWorlds < ushort.MaxValue, "b2_maxWorlds limit exceeded");
            def.CheckDef();

            var worldId = Core.NullIndex;
            for (var i = 0; i < Core.MaxWorlds; ++i)
            {
                var w = Worlds[i];
                if (w is { InUse: true })
                {
                    continue;
                }

                worldId = i;
                break;
            }

            if (worldId == Core.NullIndex)
            {
                return Box2DSharp.WorldId.NullId;
            }

            Contact.InitializeContactRegisters();

            ref var world = ref Worlds[worldId];
            var revision = world?.Revision ?? 0;

            world = new World();

            world.WorldId = (ushort)worldId;
            world.Revision = revision;
            world.InUse = true;

            world.BroadPhase = new();
            world.ConstraintGraph = new(16);

            // pools
            world.BodyIdPool = new();
            world.BodyArray = new();
            world.SolverSetArray = new();

            // add empty static, active, and disabled body sets
            world.SolverSetIdPool = new();

            // static set
            SolverSet set = new();
            set.SetIndex = world.SolverSetIdPool.AllocId();
            world.SolverSetArray.Push(set);
            Debug.Assert(world.SolverSetArray[SolverSetType.StaticSet].SetIndex == SolverSetType.StaticSet);

            // disabled set
            set = new();
            set.SetIndex = world.SolverSetIdPool.AllocId();
            world.SolverSetArray.Push(set);
            Debug.Assert(world.SolverSetArray[SolverSetType.DisabledSet].SetIndex == SolverSetType.DisabledSet);

            // awake set
            set = new();
            set.SetIndex = world.SolverSetIdPool.AllocId();
            world.SolverSetArray.Push(set);
            Debug.Assert(world.SolverSetArray[SolverSetType.AwakeSet].SetIndex == SolverSetType.AwakeSet);

            world.ShapeIdPool = new();
            world.ShapeArray = new(16);

            world.ChainIdPool = new();
            world.ChainArray = new(4);

            world.ContactIdPool = new();
            world.ContactArray = new(16);

            world.JointIdPool = new();
            world.JointArray = new(16);

            world.IslandIdPool = new();
            world.IslandArray = new(8);

            world.BodyMoveEventArray = new(4);
            world.SensorBeginEventArray = new(4);
            world.SensorEndEventArray = new(4);
            world.ContactBeginArray = new(4);
            world.ContactEndArray = new(4);
            world.ContactHitArray = new(4);

            world.StepIndex = 0;
            world.SplitIslandId = Core.NullIndex;
            world.ActiveTaskCount = 0;
            world.TaskCount = 0;
            world.Gravity = def.Gravity;
            world.HitEventThreshold = def.HitEventThreshold;
            world.RestitutionThreshold = def.RestitutionThreshold;
            world.MaxLinearVelocity = def.MaximumLinearVelocity;
            world.ContactPushOutVelocity = def.ContactPushoutVelocity;
            world.ContactHertz = def.ContactHertz;
            world.ContactDampingRatio = def.ContactDampingRatio;
            world.JointHertz = def.JointHertz;
            world.JointDampingRatio = def.JointDampingRatio;
            world.EnableSleep = def.EnableSleep;
            world.Locked = false;
            world.EnableWarmStarting = true;
            world.EnableContinuous = def.EnableContinuous;
            world.UserTreeTask = null;

            world.WorkerCount = Math.Min(def.WorkerCount, Core.MaxWorkers);
            world.EnqueueTaskFcn = def.EnqueueTask ?? DefaultAddTaskFcn;
            world.FinishTaskFcn = def.FinishTask ?? DefaultFinishTaskFcn;
            world.UserTaskContext = def.UserTaskContext;

            world.TaskContextArray = new(world.WorkerCount);
            for (int i = 0; i < world.WorkerCount; ++i)
            {
                world.TaskContextArray.Push(new TaskContext());
            }

            world.WorkerStack = new WorkerStack(def.WorkerCount);

            world.DebugBodySet = new(256);
            world.DebugJointSet = new(256);
            world.DebugContactSet = new(256);

            // add one to worldId so that 0 represents a null b2WorldId
            return new WorldId((ushort)(worldId + 1), world.Revision);
        }

        public static void DestroyWorld(WorldId worldId)
        {
            var world = GetWorldFromId(worldId);

            world.DebugBodySet = null!;
            world.DebugJointSet = null!;
            world.DebugContactSet = null!;

            for (int i = 0; i < world.WorkerCount; ++i)
            {
                world.TaskContextArray[i].ContactStateBitSet = null!;
                world.TaskContextArray[i].EnlargedSimBitSet = null!;
                world.TaskContextArray[i].AwakeIslandBitSet = null!;
            }

            world.TaskContextArray.Dispose();
            world.BodyMoveEventArray.Dispose();
            world.SensorBeginEventArray.Dispose();
            world.SensorEndEventArray.Dispose();
            world.ContactBeginArray.Dispose();
            world.ContactEndArray.Dispose();
            world.ContactHitArray.Dispose();

            int chainCapacity = world.ChainArray.Count;
            for (var i = 0; i < chainCapacity; ++i)
            {
                ref var chain = ref world.ChainArray[i];
                if (chain.Id != Core.NullIndex)
                {
                    chain.ShapeIndices = null!;
                }
                else
                {
                    Debug.Assert(chain.ShapeIndices == null);
                }
            }

            world.BodyArray.Dispose();
            world.ShapeArray.Dispose();
            world.ChainArray.Dispose();
            world.ContactArray.Dispose();
            world.JointArray.Dispose();
            world.IslandArray.Dispose();

            // The data in the solvers sets all comes from the block allocator so no
            // need to destroy the set contents.
            // todo testing
            var setCapacity = world.SolverSetArray.Count;
            for (var i = 0; i < setCapacity; ++i)
            {
                var set = world.SolverSetArray[i];
                if (set.SetIndex != Core.NullIndex)
                {
                    SolverSet.DestroySolverSet(world, i);
                }
            }

            world.SolverSetArray.Dispose();

            world.ConstraintGraph.DestroyGraph();
            world.BroadPhase.DestroyBroadPhase();

            world.BodyIdPool.DestroyIdPool();
            world.ShapeIdPool.DestroyIdPool();
            world.ChainIdPool.DestroyIdPool();
            world.ContactIdPool.DestroyIdPool();
            world.JointIdPool.DestroyIdPool();
            world.IslandIdPool.DestroyIdPool();
            world.SolverSetIdPool.DestroyIdPool();

            // Wipe world but preserve revision
            var revision = world.Revision;
            world = new();
            world.WorldId = unchecked((ushort)Core.NullIndex);
            world.Revision = (ushort)(revision + 1);
            Worlds[worldId.Index1 - 1] = world;
        }

        public static void CollideTask(int startIndex, int endIndex, int threadIndex, StepContext stepContext)
        {
            World world = stepContext.World;
            Debug.Assert(threadIndex < world.WorkerCount);
            TaskContext taskContext = world.TaskContextArray[threadIndex];
            var contactSims = stepContext.Contacts.Span;
            var shapes = world.ShapeArray.Span;
            var bodies = world.BodyArray.Span;

            Debug.Assert(startIndex < endIndex);

            for (int i = startIndex; i < endIndex; ++i)
            {
                ContactSim contactSim = contactSims[i];

                int contactId = contactSim.ContactId;

                Shape shapeA = shapes[contactSim.ShapeIdA];
                Shape shapeB = shapes[contactSim.ShapeIdB];

                // Do proxies still overlap?
                var overlap = AABB.Overlaps(shapeA.FatAABB, shapeB.FatAABB);
                if (overlap == false)
                {
                    contactSim.SimFlags |= ContactSimFlags.Disjoint;
                    contactSim.SimFlags &= ~ContactSimFlags.TouchingFlag;
                    taskContext.ContactStateBitSet.SetBit(contactId);
                }
                else
                {
                    bool wasTouching = contactSim.SimFlags.IsSet(ContactSimFlags.TouchingFlag);

                    // Update contact respecting shape/body order (A,B)
                    Body bodyA = bodies[shapeA.BodyId];
                    Body bodyB = bodies[shapeB.BodyId];
                    BodySim bodySimA = Body.GetBodySim(world, bodyA);
                    BodySim bodySimB = Body.GetBodySim(world, bodyB);

                    // avoid cache misses in b2PrepareContactsTask
                    contactSim.BodySimIndexA = bodyA.SetIndex == SolverSetType.AwakeSet ? bodyA.LocalIndex : Core.NullIndex;
                    contactSim.InvMassA = bodySimA.InvMass;
                    contactSim.InvIA = bodySimA.InvInertia;

                    contactSim.BodySimIndexB = bodyB.SetIndex == SolverSetType.AwakeSet ? bodyB.LocalIndex : Core.NullIndex;
                    contactSim.InvMassB = bodySimB.InvMass;
                    contactSim.InvIB = bodySimB.InvInertia;

                    Transform transformA = bodySimA.Transform;
                    Transform transformB = bodySimB.Transform;

                    Vec2 centerOffsetA = B2Math.RotateVector(transformA.Q, bodySimA.LocalCenter);
                    Vec2 centerOffsetB = B2Math.RotateVector(transformB.Q, bodySimB.LocalCenter);

                    // This updates solid contacts and sensors
                    bool touching = Contact.UpdateContact(world, contactSim, shapeA, transformA, centerOffsetA, shapeB, transformB, centerOffsetB);

                    // State changes that affect island connectivity. Also contact and sensor events.
                    if (touching == true && wasTouching == false)
                    {
                        contactSim.SimFlags |= ContactSimFlags.StartedTouching;
                        taskContext.ContactStateBitSet.SetBit(contactId);
                    }
                    else if (touching == false && wasTouching == true)
                    {
                        contactSim.SimFlags |= ContactSimFlags.StoppedTouching;
                        taskContext.ContactStateBitSet.SetBit(contactId);
                    }
                }
            }
        }

        private static void AddNonTouchingContact(World world, Contact contact, ContactSim contactSim)
        {
            Debug.Assert(contact.SetIndex == SolverSetType.AwakeSet);
            SolverSet set = world.SolverSetArray[SolverSetType.AwakeSet];
            contact.ColorIndex = Core.NullIndex;
            contact.LocalIndex = set.Contacts.Count;

            ContactSim newContactSim = set.Contacts.AddContact();
            contactSim.CopyTo(newContactSim);
        }

        private static void RemoveNonTouchingContact(World world, int setIndex, int localIndex)
        {
            world.SolverSetArray.CheckIndex(setIndex);
            SolverSet set = world.SolverSetArray[setIndex];
            int movedIndex = set.Contacts.RemoveContact(localIndex);
            if (movedIndex == Core.NullIndex)
            {
                return;
            }

            ContactSim movedContactSim = set.Contacts.Data[localIndex];
            world.ContactArray.CheckIndex(movedContactSim.ContactId);
            Contact movedContact = world.ContactArray[movedContactSim.ContactId];
            Debug.Assert(movedContact.SetIndex == setIndex);
            Debug.Assert(movedContact.LocalIndex == movedIndex);
            Debug.Assert(movedContact.ColorIndex == Core.NullIndex);
            movedContact.LocalIndex = localIndex;
        }

        /// <summary>
        /// Narrow-phase collision
        /// 近距离碰撞
        /// </summary>
        /// <param name="context"></param>
        private static void Collide(StepContext context)
        {
            World world = context.World;

            Debug.Assert(world.WorkerCount > 0);

            // Tasks that can be done in parallel with the narrow-phase
            // - rebuild the collision tree for dynamic and kinematic bodies to keep their query performance good
            // 对动态和运动刚体重建碰撞树，以保持查询性能
            var treeTask = new ManualResetEventSlim(false);
            world.UserTreeTask = treeTask;
            ThreadPool.QueueUserWorkItem(
                _ =>
                {
                    world.BroadPhase.RebuildTrees();
                    treeTask.Set();
                });

            world.TaskCount += 1;
            world.ActiveTaskCount += world.UserTreeTask == null ? 0 : 1;

            // gather contacts into a single array for easier parallel-for
            // 统计所有着色图的接触点
            int contactCount = 0;
            var graphColors = world.ConstraintGraph.Colors;
            for (int i = 0; i < Core.GraphColorCount; ++i)
            {
                contactCount += graphColors[i].Contacts.Count;
            }

            int nonTouchingCount = world.SolverSetArray[SolverSetType.AwakeSet].Contacts.Count; // 无触碰计数？
            contactCount += nonTouchingCount;

            if (contactCount == 0)
            {
                return;
            }

            // todo Pool
            var contactSims = B2ArrayPool<ContactSim>.Shared.Rent(contactCount); // new ContactSim[contactCount];

            int contactIndex = 0;
            for (int i = 0; i < Core.GraphColorCount; ++i)
            {
                // 把所有着色图中的接触点仿真数据放入到新建仿真数组
                GraphColor color = graphColors[i];
                int count = color.Contacts.Count;
                var baseSim = color.Contacts.Span;
                for (int j = 0; j < count; ++j)
                {
                    contactSims[contactIndex] = baseSim[j];
                    contactIndex += 1;
                }
            }

            {
                // 把活跃解算集合的接触点也放入新建仿真数组
                var baseSim = world.SolverSetArray[SolverSetType.AwakeSet].Contacts.Span;
                for (int i = 0; i < nonTouchingCount; ++i)
                {
                    contactSims[contactIndex] = baseSim[i];
                    contactIndex += 1;
                }
            }

            Debug.Assert(contactIndex == contactCount);

            // 本次上下文中要处理的接触点仿真数据
            context.Contacts = contactSims;

            // Contact bit set on ids because contact pointers are unstable as they move between touching and not touching.
            int contactIdCapacity = world.ContactIdPool.GetIdCapacity();
            for (int i = 0; i < world.WorkerCount; ++i)
            {
                world.TaskContextArray[i].ContactStateBitSet.SetBitCountAndClear(contactIdCapacity);
            }

            // Task should take at least 40us on a 4GHz CPU (10K cycles)
            // 并行执行碰撞任务
            var stack = world.WorkerStack;
            Parallel.ForEach(
                Partitioner.Create(0, contactCount),
                new ParallelOptions { MaxDegreeOfParallelism = world.WorkerCount },
                range =>
                {
                    if (!stack.TryPop(out var index))
                    {
                        throw new Exception("No more worker");
                    }

                    CollideTask(range.Item1, range.Item2, index, context);
                    stack.Push(index);
                });

            // CollideTask(0, contactCount, 0, context);
            world.TaskCount += 1;

            context.Contacts = null;
            B2ArrayPool<ContactSim>.Shared.Return(contactSims);

            // Serially update contact state

            // Bitwise OR all contact bits
            // 对所有任务上下文的接触状态位标志进行与操作，存在0号任务上下文中
            BitSet bitSet = world.TaskContextArray[0].ContactStateBitSet;
            for (int i = 1; i < world.WorkerCount; ++i)
            {
                bitSet.InPlaceUnion(world.TaskContextArray[i].ContactStateBitSet);
            }

            var contacts = world.ContactArray;
            SolverSet awakeSet = world.SolverSetArray[SolverSetType.AwakeSet];

            var shapes = world.ShapeArray;
            var worldId = world.WorldId;

            // Process contact state changes. Iterate over set bits
            for (uint k = 0; k < bitSet.BlockCount; ++k)
            {
                var bits = bitSet.Bits[k];
                while (bits != 0)
                {
                    int ctz = BitTool.CTZ64(bits);       // 从低位统计0位的数量
                    int contactId = (int)(64 * k + ctz); // 按bit索引计算出接触点Id

                    contacts.CheckIndex(contactId);

                    Contact contact = contacts[contactId];
                    Debug.Assert(contact.SetIndex == SolverSetType.AwakeSet);

                    int colorIndex = contact.ColorIndex;
                    int localIndex = contact.LocalIndex;

                    ContactSim contactSim;
                    if (colorIndex != Core.NullIndex)
                    {
                        // contact lives in constraint graph
                        Debug.Assert(0 <= colorIndex && colorIndex < Core.GraphColorCount);
                        GraphColor color = graphColors[colorIndex];
                        Debug.Assert(0 <= localIndex && localIndex < color.Contacts.Count);
                        contactSim = color.Contacts.Data[localIndex];
                    }
                    else
                    {
                        Debug.Assert(0 <= localIndex && localIndex < awakeSet.Contacts.Count);
                        contactSim = awakeSet.Contacts.Data[localIndex];
                    }

                    Shape shapeA = shapes[contact.ShapeIdA];
                    Shape shapeB = shapes[contact.ShapeIdB];
                    ShapeId shapeIdA = (shapeA.Id + 1, worldId, shapeA.Revision);
                    ShapeId shapeIdB = (shapeB.Id + 1, worldId, shapeB.Revision);
                    var flags = contact.Flags;
                    var simFlags = contactSim.SimFlags;

                    if (simFlags.IsSet(ContactSimFlags.Disjoint))
                    {
                        // Was touching?
                        if ((flags.IsSet(ContactFlags.ContactTouchingFlag)) && (flags.IsSet(ContactFlags.ContactEnableContactEvents)))
                        {
                            ContactEndTouchEvent evt = new(shapeIdA, shapeIdB);
                            world.ContactEndArray.Push(evt);
                        }

                        // Bounding boxes no longer overlap
                        contact.Flags &= ~ContactFlags.ContactTouchingFlag;
                        Contact.DestroyContact(world, contact, false);
                    }
                    else if (simFlags.IsSet(ContactSimFlags.StartedTouching))
                    {
                        Debug.Assert(contact.IslandId == Core.NullIndex);
                        if ((flags & ContactFlags.ContactSensorFlag) != 0)
                        {
                            // Contact is a sensor
                            if ((flags & ContactFlags.ContactEnableSensorEvents) != 0)
                            {
                                if (shapeA.IsSensor)
                                {
                                    SensorBeginTouchEvent evt = new(shapeIdA, shapeIdB);
                                    world.SensorBeginEventArray.Push(evt);
                                }

                                if (shapeB.IsSensor)
                                {
                                    SensorBeginTouchEvent evt = new(shapeIdB, shapeIdA);
                                    world.SensorBeginEventArray.Push(evt);
                                }
                            }

                            contactSim.SimFlags &= ~ ContactSimFlags.StartedTouching;
                            contact.Flags |= ContactFlags.ContactSensorTouchingFlag;
                        }
                        else
                        {
                            // Contact is solid
                            if (flags.IsSet(ContactFlags.ContactEnableContactEvents))
                            {
                                ContactBeginTouchEvent evt = new(shapeIdA, shapeIdB);
                                world.ContactBeginArray.Push(evt);
                            }

                            Debug.Assert(contactSim.Manifold.PointCount > 0);
                            Debug.Assert(contact.SetIndex == SolverSetType.AwakeSet);

                            // Link first because this wakes colliding bodies and ensures the body sims
                            // are in the correct place.
                            contact.Flags |= ContactFlags.ContactTouchingFlag;
                            Island.LinkContact(world, ref contact);

                            // Make sure these didn't change
                            Debug.Assert(contact.ColorIndex == Core.NullIndex);
                            Debug.Assert(contact.LocalIndex == localIndex);

                            // Contact sim pointer may have become orphaned due to awake set growth,
                            // so I just need to refresh it.
                            Debug.Assert(0 <= localIndex && localIndex < awakeSet.Contacts.Count);
                            contactSim = awakeSet.Contacts.Data[localIndex];

                            contactSim.SimFlags &= ~ ContactSimFlags.StartedTouching;

                            ConstraintGraph.AddContactToGraph(world, contactSim, contact);
                            RemoveNonTouchingContact(world, SolverSetType.AwakeSet, localIndex);
                        }
                    }
                    else if (simFlags.IsSet(ContactSimFlags.StoppedTouching))
                    {
                        contactSim.SimFlags &= ~ ContactSimFlags.StoppedTouching;

                        if (flags.IsSet(ContactFlags.ContactSensorFlag))
                        {
                            // Contact is a sensor
                            contact.Flags &= ~ContactFlags.ContactSensorTouchingFlag;

                            if (flags.IsSet(ContactFlags.ContactEnableSensorEvents))
                            {
                                if (shapeA.IsSensor)
                                {
                                    SensorEndTouchEvent evt = new(shapeIdA, shapeIdB);
                                    world.SensorEndEventArray.Push(evt);
                                }

                                if (shapeB.IsSensor)
                                {
                                    SensorEndTouchEvent evt = new(shapeIdB, shapeIdA)
                                        ;
                                    world.SensorEndEventArray.Push(evt);
                                }
                            }
                        }
                        else
                        {
                            // Contact is solid
                            contact.Flags &= ~ContactFlags.ContactTouchingFlag;

                            if (contact.Flags.IsSet(ContactFlags.ContactEnableContactEvents))
                            {
                                ContactEndTouchEvent evt = new(shapeIdA, shapeIdB);
                                world.ContactEndArray.Push(evt);
                            }

                            Debug.Assert(contactSim.Manifold.PointCount == 0);

                            Island.UnlinkContact(world, contact);
                            int bodyIdA = contact.Edges[0].BodyId;
                            int bodyIdB = contact.Edges[1].BodyId;

                            AddNonTouchingContact(world, contact, contactSim);
                            ConstraintGraph.RemoveContactFromGraph(world, bodyIdA, bodyIdB, colorIndex, localIndex);
                        }
                    }

                    // Clear the smallest set bit
                    bits &= bits - 1;
                }
            }

            SolverSet.ValidateSolverSets(world);
            SolverSet.ValidateContacts(world);
        }

        public static void Step(WorldId worldId, float timeStep, int subStepCount)
        {
            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return;
            }

            // Prepare to capture events
            // Ensure user does not access stale data if there is an early return
            world.BodyMoveEventArray.Clear();
            world.SensorBeginEventArray.Clear();
            world.SensorEndEventArray.Clear();
            world.ContactBeginArray.Clear();
            world.ContactEndArray.Clear();
            world.ContactHitArray.Clear();

            world.Profile = new Profile();

            if (timeStep == 0.0f)
            {
                // todo would be useful to still process collision while paused
                return;
            }

            world.Locked = true;
            world.ActiveTaskCount = 0;
            world.TaskCount = 0;

            var stepTimer = Stopwatch.StartNew();

            // Update collision pairs and create contacts
            // 更新粗检测碰撞配对，创建接触点
            {
                var t1 = Stopwatch.GetTimestamp();
                BroadPhase.UpdateBroadPhasePairs(world);

                world.Profile.Pairs = (float)StopwatchHelper.GetElapsedTime(t1).TotalMilliseconds;
            }

            StepContext context = new();
            context.World = world;
            context.Dt = timeStep;
            context.SubStepCount = Math.Max(1, subStepCount);

            if (timeStep > 0.0f)
            {
                context.InvDt = 1.0f / timeStep;
                context.H = timeStep / context.SubStepCount;
                context.InvH = context.SubStepCount * context.InvDt;
            }
            else
            {
                context.InvDt = 0.0f;
                context.H = 0.0f;
                context.InvH = 0.0f;
            }

            world.InvH = context.InvH;

            // Hertz values get reduced for large time steps
            float contactHertz = Math.Min(world.ContactHertz, 0.25f * context.InvH);
            float jointHertz = Math.Min(world.JointHertz, 0.125f * context.InvH);

            context.ContactSoftness = Softness.MakeSoft(contactHertz, world.ContactDampingRatio, context.H);
            context.StaticSoftness = Softness.MakeSoft(2.0f * contactHertz, world.ContactDampingRatio, context.H);
            context.JointSoftness = Softness.MakeSoft(jointHertz, world.JointDampingRatio, context.H);

            context.RestitutionThreshold = world.RestitutionThreshold;
            context.MaxLinearVelocity = world.MaxLinearVelocity;
            context.EnableWarmStarting = world.EnableWarmStarting;

            // Update contacts
            // 近距离精细碰撞，更新接触点
            {
                var t1 = Stopwatch.GetTimestamp();
                Collide(context);
                world.Profile.Collide = (float)StopwatchHelper.GetElapsedTime(t1).TotalMilliseconds;
            }

            // Integrate velocities, solve velocity constraints, and integrate positions.
            // 速度积分，求解速度约束，然后位置积分
            if (context.Dt > 0.0f)
            {
                var t1 = Stopwatch.GetTimestamp();
                Solver.Solve(world, context);
                world.Profile.Solve = (float)StopwatchHelper.GetElapsedTime(t1).TotalMilliseconds;
            }

            world.Locked = false;

            world.Profile.Step = stepTimer.ElapsedMilliseconds;

            // Make sure all tasks that were started were also finished
            Debug.Assert(world.ActiveTaskCount == 0);
        }

        public static void DrawShape(DebugDrawBase draw, Shape shape, Transform xf, B2HexColor color)
        {
            switch (shape.Type)
            {
            case ShapeType.CapsuleShape:
            {
                Capsule capsule = shape.Union.Capsule;
                Vec2 p1 = B2Math.TransformPoint(xf, capsule.Center1);
                Vec2 p2 = B2Math.TransformPoint(xf, capsule.Center2);
                draw.DrawSolidCapsule(p1, p2, capsule.Radius, color, draw.Context);
            }
                break;

            case ShapeType.CircleShape:
            {
                Circle circle = shape.Union.Circle;
                xf.P = B2Math.TransformPoint(xf, circle.Center);
                draw.DrawSolidCircle(xf, circle.Radius, color, draw.Context);
            }
                break;

            case ShapeType.PolygonShape:
            {
                Polygon poly = shape.Union.Polygon;
                draw.DrawSolidPolygon(xf, poly.Vertices, poly.Count, poly.Radius, color, draw.Context);
            }
                break;

            case ShapeType.SegmentShape:
            {
                Segment segment = shape.Union.Segment;
                Vec2 p1 = B2Math.TransformPoint(xf, segment.Point1);
                Vec2 p2 = B2Math.TransformPoint(xf, segment.Point2);
                draw.DrawSegment(p1, p2, color, draw.Context);
            }
                break;

            case ShapeType.ChainSegmentShape:
            {
                Segment segment = shape.Union.ChainSegment.Segment;
                Vec2 p1 = B2Math.TransformPoint(xf, segment.Point1);
                Vec2 p2 = B2Math.TransformPoint(xf, segment.Point2);
                draw.DrawSegment(p1, p2, color, draw.Context);
                draw.DrawPoint(p2, 4.0f, color, draw.Context);
                draw.DrawSegment(p1, B2Math.Lerp(p1, p2, 0.1f), B2HexColor.PaleGreen, draw.Context);
            }
                break;
            default:
                throw new InvalidEnumArgumentException($"Shape type {shape.Type} is invalid");
            }
        }

        private class DrawContext
        {
            public World World;

            public DebugDrawBase Draw;

            public DrawContext(World world, DebugDrawBase draw)
            {
                World = world;
                Draw = draw;
            }
        };

        private static readonly TreeQueryCallbackFcn _drawQueryCallback = (proxyId, shapeId, context) =>
        {
            var drawContext = (DrawContext)context;
            World world = drawContext.World;
            DebugDrawBase draw = drawContext.Draw;

            world.ShapeArray.CheckId(shapeId);
            Shape shape = world.ShapeArray[shapeId];

            world.DebugBodySet.SetBit(shape.BodyId);

            if (draw.DrawShapes)
            {
                world.BodyArray.CheckId(shape.BodyId);
                Body body = world.BodyArray[shape.BodyId];
                BodySim bodySim = Body.GetBodySim(world, body);

                B2HexColor color;

                if (shape.CustomColor != 0)
                {
                    color = (B2HexColor)shape.CustomColor;
                }
                else if (body.Type == BodyType.DynamicBody && bodySim.Mass == 0.0f)
                {
                    // Bad body
                    color = B2HexColor.Red;
                }
                else if (body.SetIndex == SolverSetType.DisabledSet)
                {
                    color = B2HexColor.SlateGray;
                }
                else if (shape.IsSensor)
                {
                    color = B2HexColor.Wheat;
                }
                else if (bodySim.IsBullet && body.SetIndex == SolverSetType.AwakeSet)
                {
                    color = B2HexColor.Turquoise;
                }
                else if (body.IsSpeedCapped)
                {
                    color = B2HexColor.Yellow;
                }
                else if (bodySim.IsFast)
                {
                    color = B2HexColor.Salmon;
                }
                else if (body.Type == BodyType.StaticBody)
                {
                    color = B2HexColor.PaleGreen;
                }
                else if (body.Type == BodyType.KinematicBody)
                {
                    color = B2HexColor.RoyalBlue;
                }
                else if (body.SetIndex == SolverSetType.AwakeSet)
                {
                    color = B2HexColor.Pink;
                }
                else
                {
                    color = B2HexColor.Gray;
                }

                DrawShape(draw, shape, bodySim.Transform, color);
            }

            if (draw.DrawAABBs)
            {
                AABB aabb = shape.FatAABB;

                Span<Vec2> vs = stackalloc Vec2[4]
                {
                    (aabb.LowerBound.X, aabb.LowerBound.Y),
                    (aabb.UpperBound.X, aabb.LowerBound.Y),
                    (aabb.UpperBound.X, aabb.UpperBound.Y),
                    (aabb.LowerBound.X, aabb.UpperBound.Y)
                };

                draw.DrawPolygon(vs, 4, B2HexColor.Gold, draw.Context);
            }

            return true;
        };

        // todo this has varying order for moving shapes, causing flicker when overlapping shapes are moving
        // solution: display order by shape id modulus 3, keep 3 buckets in GLSolid* and flush in 3 passes.
        private static void DrawWithBounds(World world, DebugDrawBase draw)
        {
            Debug.Assert(draw.DrawingBounds.IsValid);

            const float ImpulseScale = 1.0f;
            const float AxisScale = 0.3f;
            const B2HexColor speculativeColor = B2HexColor.Gray3;
            const B2HexColor addColor = B2HexColor.Green;
            const B2HexColor persistColor = B2HexColor.Blue;
            const B2HexColor normalColor = B2HexColor.Gray9;
            const B2HexColor impulseColor = B2HexColor.Magenta;
            const B2HexColor frictionColor = B2HexColor.Yellow;

            Span<B2HexColor> graphColors = stackalloc B2HexColor[]
            {
                B2HexColor.Red, B2HexColor.Orange, B2HexColor.Yellow, B2HexColor.Green,
                B2HexColor.Cyan, B2HexColor.Blue, B2HexColor.Violet, B2HexColor.Pink,
                B2HexColor.Chocolate, B2HexColor.Goldenrod, B2HexColor.Coral, B2HexColor.Black
            };

            int bodyCapacity = world.BodyIdPool.GetIdCapacity();
            world.DebugBodySet.SetBitCountAndClear(bodyCapacity);

            int jointCapacity = world.JointIdPool.GetIdCapacity();
            world.DebugJointSet.SetBitCountAndClear(jointCapacity);

            int contactCapacity = world.ContactIdPool.GetIdCapacity();
            world.DebugContactSet.SetBitCountAndClear(contactCapacity);

            DrawContext drawContext = new(world, draw);

            for (int i = 0; i < Core.BodyTypeCount; ++i)
            {
                world.BroadPhase.Trees[i]
                     .Query(
                          draw.DrawingBounds,
                          Core.DefaultMaskBits,
                          _drawQueryCallback,
                          drawContext);
            }

            var wordCount = world.DebugBodySet.BlockCount;
            var bits = world.DebugBodySet.Bits;
            for (int k = 0; k < wordCount; ++k)
            {
                var word = bits[k];
                while (word != 0)
                {
                    int ctz = BitTool.CTZ64(word);
                    int bodyId = 64 * k + ctz;

                    world.BodyArray.CheckId(bodyId);
                    Body body = world.BodyArray[bodyId];

                    if (draw.DrawMass && body.Type == BodyType.DynamicBody)
                    {
                        Vec2 offset = (0.1f, 0.1f);
                        BodySim bodySim = Body.GetBodySim(world, body);

                        Transform transform = (bodySim.Center, bodySim.Transform.Q);
                        draw.DrawTransform(transform, draw.Context);

                        Vec2 p = B2Math.TransformPoint(transform, offset);

                        draw.DrawString(p, $"  {bodySim.Mass:F2}", draw.Context);
                    }

                    if (draw.DrawJoints)
                    {
                        int jointKey = body.HeadJointKey;
                        while (jointKey != Core.NullIndex)
                        {
                            int jointId = jointKey >> 1;
                            int edgeIndex = jointKey & 1;
                            Joint joint = world.JointArray[jointId];

                            // avoid double draw
                            if (world.DebugJointSet.GetBit(jointId) == false)
                            {
                                Joint.DrawJoint(draw, world, joint);
                                world.DebugJointSet.SetBit(jointId);
                            }
                            else
                            {
                                // todo testing
                                edgeIndex += 0;
                            }

                            jointKey = joint.Edges[edgeIndex].NextKey;
                        }
                    }

                    float linearSlop = Core.LinearSlop;
                    if (draw.DrawContacts && body is { Type: BodyType.DynamicBody, SetIndex: SolverSetType.AwakeSet })
                    {
                        int contactKey = body.HeadContactKey;
                        while (contactKey != Core.NullIndex)
                        {
                            int contactId = contactKey >> 1;
                            int edgeIndex = contactKey & 1;
                            Contact contact = world.ContactArray[contactId];
                            contactKey = contact.Edges[edgeIndex].NextKey;

                            if (contact.SetIndex != SolverSetType.AwakeSet || contact.ColorIndex == Core.NullIndex)
                            {
                                continue;
                            }

                            // avoid double draw
                            if (world.DebugContactSet.GetBit(contactId) == false)
                            {
                                Debug.Assert(contact.ColorIndex is >= 0 and < Core.GraphColorCount);

                                GraphColor gc = world.ConstraintGraph.Colors[contact.ColorIndex];
                                Debug.Assert(0 <= contact.LocalIndex && contact.LocalIndex < gc.Contacts.Count);

                                ContactSim contactSim = gc.Contacts.Data[contact.LocalIndex];
                                int pointCount = contactSim.Manifold.PointCount;
                                Vec2 normal = contactSim.Manifold.Normal;

                                for (int j = 0; j < pointCount; ++j)
                                {
                                    ref readonly ManifoldPoint point = ref contactSim.Manifold.Points[j];

                                    if (draw.DrawGraphColors)
                                    {
                                        // graph color
                                        float pointSize = contact.ColorIndex == Core.OverflowIndex ? 7.5f : 5.0f;
                                        draw.DrawPoint(point.Point, pointSize, graphColors[contact.ColorIndex], draw.Context);

                                        // g_draw.DrawString(point.position, "%d", point.color);
                                    }
                                    else if (point.Separation > linearSlop)
                                    {
                                        // Speculative
                                        draw.DrawPoint(point.Point, 5.0f, speculativeColor, draw.Context);
                                    }
                                    else if (point.Persisted == false)
                                    {
                                        // Add
                                        draw.DrawPoint(point.Point, 10.0f, addColor, draw.Context);
                                    }
                                    else if (point.Persisted == true)
                                    {
                                        // Persist
                                        draw.DrawPoint(point.Point, 5.0f, persistColor, draw.Context);
                                    }

                                    if (draw.DrawContactNormals)
                                    {
                                        Vec2 p1 = point.Point;
                                        Vec2 p2 = B2Math.MulAdd(p1, AxisScale, normal);
                                        draw.DrawSegment(p1, p2, normalColor, draw.Context);
                                    }
                                    else if (draw.DrawContactImpulses)
                                    {
                                        Vec2 p1 = point.Point;
                                        Vec2 p2 = B2Math.MulAdd(p1, ImpulseScale * point.NormalImpulse, normal);
                                        draw.DrawSegment(p1, p2, impulseColor, draw.Context);
                                        draw.DrawString(p1, (1000.0f * point.NormalImpulse).ToString("F1"), draw.Context);
                                    }

                                    if (draw.DrawFrictionImpulses)
                                    {
                                        Vec2 tangent = B2Math.RightPerp(normal);
                                        Vec2 p1 = point.Point;
                                        Vec2 p2 = B2Math.MulAdd(p1, ImpulseScale * point.TangentImpulse, tangent);
                                        draw.DrawSegment(p1, p2, frictionColor, draw.Context);
                                        draw.DrawString(p1, (1000.0f * point.TangentImpulse).ToString("F1"), draw.Context);
                                    }
                                }

                                world.DebugContactSet.SetBit(contactId);
                            }
                            else
                            {
                                // todo testing
                                edgeIndex += 0;
                            }

                            contactKey = contact.Edges[edgeIndex].NextKey;
                        }
                    }

                    // Clear the smallest set bit
                    word = word & (word - 1);
                }
            }
        }

        public static void Draw(WorldId worldId, DebugDrawBase draw)
        {
            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return;
            }

            // todo it seems bounds drawing is fast enough for regular usage
            if (draw.UseDrawingBounds)
            {
                DrawWithBounds(world, draw);
                return;
            }

            if (draw.DrawShapes)
            {
                int setCount = (world.SolverSetArray).Count;
                for (int setIndex = 0; setIndex < setCount; ++setIndex)
                {
                    SolverSet set = world.SolverSetArray[setIndex];
                    int bodyCount = set.Sims.Count;
                    for (int bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex)
                    {
                        BodySim bodySim = set.Sims.Data[bodyIndex];
                        world.BodyArray.CheckIndex(bodySim.BodyId);
                        Body body = world.BodyArray[bodySim.BodyId];
                        Debug.Assert(body.SetIndex == setIndex);

                        Transform xf = bodySim.Transform;
                        int shapeId = body.HeadShapeId;
                        while (shapeId != Core.NullIndex)
                        {
                            Shape shape = world.ShapeArray[shapeId];
                            B2HexColor color;

                            if (shape.CustomColor != 0)
                            {
                                color = (B2HexColor)shape.CustomColor;
                            }
                            else if (body.Type == BodyType.DynamicBody && bodySim.Mass == 0.0f)
                            {
                                // Bad body
                                color = B2HexColor.Red;
                            }
                            else if (body.SetIndex == SolverSetType.DisabledSet)
                            {
                                color = B2HexColor.SlateGray;
                            }
                            else if (shape.IsSensor)
                            {
                                color = B2HexColor.Wheat;
                            }
                            else if (bodySim.IsBullet && body.SetIndex == SolverSetType.AwakeSet)
                            {
                                color = B2HexColor.Turquoise;
                            }
                            else if (body.IsSpeedCapped)
                            {
                                color = B2HexColor.Yellow;
                            }
                            else if (bodySim.IsFast)
                            {
                                color = B2HexColor.Salmon;
                            }
                            else if (body.Type == BodyType.StaticBody)
                            {
                                color = B2HexColor.PaleGreen;
                            }
                            else if (body.Type == BodyType.KinematicBody)
                            {
                                color = B2HexColor.RoyalBlue;
                            }
                            else if (body.SetIndex == SolverSetType.AwakeSet)
                            {
                                color = B2HexColor.Pink;
                            }
                            else
                            {
                                color = B2HexColor.Gray;
                            }

                            DrawShape(draw, shape, xf, color);
                            shapeId = shape.NextShapeId;
                        }
                    }
                }
            }

            if (draw.DrawJoints)
            {
                int count = (world.JointArray).Count;
                for (int i = 0; i < count; ++i)
                {
                    Joint joint = world.JointArray[i];
                    if (joint.SetIndex == Core.NullIndex)
                    {
                        continue;
                    }

                    Joint.DrawJoint(draw, world, joint);
                }
            }

            if (draw.DrawAABBs)
            {
                B2HexColor color = B2HexColor.Gold;

                int setCount = (world.SolverSetArray).Count;
                Span<Vec2> vs = stackalloc Vec2[4];
                for (int setIndex = 0; setIndex < setCount; ++setIndex)
                {
                    SolverSet set = world.SolverSetArray[setIndex];
                    int bodyCount = set.Sims.Count;
                    for (int bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex)
                    {
                        BodySim bodySim = set.Sims.Data[bodyIndex];

                        draw.DrawString(bodySim.Center, bodySim.BodyId.ToString(), draw.Context);

                        world.BodyArray.CheckIndex(bodySim.BodyId);
                        Body body = world.BodyArray[bodySim.BodyId];
                        Debug.Assert(body.SetIndex == setIndex);

                        int shapeId = body.HeadShapeId;
                        while (shapeId != Core.NullIndex)
                        {
                            Shape shape = world.ShapeArray[shapeId];
                            AABB aabb = shape.FatAABB;

                            vs[0] = (aabb.LowerBound.X, aabb.LowerBound.Y);
                            vs[1] = (aabb.UpperBound.X, aabb.LowerBound.Y);
                            vs[2] = (aabb.UpperBound.X, aabb.UpperBound.Y);
                            vs[3] = (aabb.LowerBound.X, aabb.UpperBound.Y);

                            draw.DrawPolygon(vs, 4, color, draw.Context);

                            shapeId = shape.NextShapeId;
                        }
                    }
                }
            }

            if (draw.DrawMass)
            {
                Vec2 offset = (0.1f, 0.1f);
                int setCount = (world.SolverSetArray).Count;
                for (int setIndex = 0; setIndex < setCount; ++setIndex)
                {
                    SolverSet set = world.SolverSetArray[setIndex];
                    int bodyCount = set.Sims.Count;
                    for (int bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex)
                    {
                        BodySim bodySim = set.Sims.Data[bodyIndex];

                        Transform transform = (bodySim.Center, bodySim.Transform.Q);
                        draw.DrawTransform(transform, draw.Context);

                        Vec2 p = B2Math.TransformPoint(transform, offset);

                        draw.DrawString(p, $"  {bodySim.Mass:F2}", draw.Context);
                    }
                }
            }

            if (draw.DrawContacts)
            {
                const float k_impulseScale = 1.0f;
                const float k_axisScale = 0.3f;
                float linearSlop = Core.LinearSlop;

                B2HexColor speculativeColor = B2HexColor.Gray3;
                B2HexColor addColor = B2HexColor.Green;
                B2HexColor persistColor = B2HexColor.Blue;
                B2HexColor normalColor = B2HexColor.Gray9;
                B2HexColor impulseColor = B2HexColor.Magenta;
                B2HexColor frictionColor = B2HexColor.Yellow;

                // [Core.b2_graphColorCount]
                Span<B2HexColor> colors = stackalloc B2HexColor[]
                {
                    B2HexColor.Red, B2HexColor.Orange, B2HexColor.Yellow, B2HexColor.Green,
                    B2HexColor.Cyan, B2HexColor.Blue, B2HexColor.Violet, B2HexColor.Pink,
                    B2HexColor.Chocolate, B2HexColor.Goldenrod, B2HexColor.Coral, B2HexColor.Black
                };

                for (int colorIndex = 0; colorIndex < Core.GraphColorCount; ++colorIndex)
                {
                    GraphColor graphColor = world.ConstraintGraph.Colors[colorIndex];

                    int contactCount = graphColor.Contacts.Count;
                    for (int contactIndex = 0; contactIndex < contactCount; ++contactIndex)
                    {
                        ContactSim contact = graphColor.Contacts.Data[contactIndex];
                        int pointCount = contact.Manifold.PointCount;
                        Vec2 normal = contact.Manifold.Normal;

                        for (int j = 0; j < pointCount; ++j)
                        {
                            ref readonly ManifoldPoint point = ref contact.Manifold.Points[j];

                            if (draw.DrawGraphColors && colorIndex >= 0)
                            {
                                // graph color
                                float pointSize = colorIndex == Core.OverflowIndex ? 7.5f : 5.0f;
                                draw.DrawPoint(point.Point, pointSize, colors[colorIndex], draw.Context);

                                // g_draw.DrawString(point.position, "%d", point.color);
                            }
                            else if (point.Separation > linearSlop)
                            {
                                // Speculative
                                draw.DrawPoint(point.Point, 5.0f, speculativeColor, draw.Context);
                            }
                            else if (point.Persisted == false)
                            {
                                // Add
                                draw.DrawPoint(point.Point, 10.0f, addColor, draw.Context);
                            }
                            else if (point.Persisted == true)
                            {
                                // Persist
                                draw.DrawPoint(point.Point, 5.0f, persistColor, draw.Context);
                            }

                            if (draw.DrawContactNormals)
                            {
                                Vec2 p1 = point.Point;
                                Vec2 p2 = B2Math.MulAdd(p1, k_axisScale, normal);
                                draw.DrawSegment(p1, p2, normalColor, draw.Context);
                            }
                            else if (draw.DrawContactImpulses)
                            {
                                Vec2 p1 = point.Point;
                                Vec2 p2 = B2Math.MulAdd(p1, k_impulseScale * point.NormalImpulse, normal);
                                draw.DrawSegment(p1, p2, impulseColor, draw.Context);
                                draw.DrawString(p1, (1000.0f * point.NormalImpulse).ToString("F2"), draw.Context);
                            }

                            if (draw.DrawFrictionImpulses)
                            {
                                Vec2 tangent = B2Math.RightPerp(normal);
                                Vec2 p1 = point.Point;
                                Vec2 p2 = B2Math.MulAdd(p1, k_impulseScale * point.TangentImpulse, tangent);
                                draw.DrawSegment(p1, p2, frictionColor, draw.Context);
                                draw.DrawString(p1, point.NormalImpulse.ToString("F2"), draw.Context);
                            }
                        }
                    }
                }
            }
        }

        public static BodyEvents GetBodyEvents(WorldId worldId)
        {
            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return new BodyEvents();
            }

            int count = world.BodyMoveEventArray.Count;
            BodyEvents events = new(world.BodyMoveEventArray, count);
            return events;
        }

        public static SensorEvents GetSensorEvents(WorldId worldId)
        {
            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return new SensorEvents();
            }

            int beginCount = (world.SensorBeginEventArray).Count;
            int endCount = (world.SensorEndEventArray).Count;

            SensorEvents events = new(world.SensorBeginEventArray, world.SensorEndEventArray, beginCount, endCount);
            return events;
        }

        public static ContactEvents GetContactEvents(WorldId worldId)
        {
            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return new ContactEvents();
            }

            int beginCount = (world.ContactBeginArray).Count;
            int endCount = (world.ContactEndArray).Count;
            int hitCount = (world.ContactHitArray).Count;

            ContactEvents events = new(world.ContactBeginArray, world.ContactEndArray, world.ContactHitArray, beginCount, endCount, hitCount);

            return events;
        }

        public static bool World_IsValid(WorldId id)
        {
            if (id.Index1 < 1 || Core.MaxWorlds < id.Index1)
            {
                return false;
            }

            World? world = Worlds[id.Index1 - 1];

            if (world == null || world.WorldId != id.Index1 - 1)
            {
                // world is not allocated
                return false;
            }

            return id.Revision == world.Revision;
        }

        public static bool Body_IsValid(BodyId id)
        {
            if (Core.MaxWorlds <= id.World0)
            {
                // invalid world
                return false;
            }

            World world = Worlds[id.World0] ?? throw new NullReferenceException($"World {id.World0} is null");
            if (world.WorldId != id.World0)
            {
                // world is free
                return false;
            }

            if (id.Index1 < 1 || (world.BodyArray).Count < id.Index1)
            {
                // invalid index
                return false;
            }

            Body body = world.BodyArray[(id.Index1 - 1)];
            if (body.SetIndex == Core.NullIndex)
            {
                // this was freed
                return false;
            }

            Debug.Assert(body.LocalIndex != Core.NullIndex);

            if (body.Revision != id.Revision)
            {
                // this id is orphaned
                return false;
            }

            return true;
        }

        public static bool ShapeIsValid(ShapeId id)
        {
            if (Core.MaxWorlds <= id.World0)
            {
                return false;
            }

            World world = Worlds[id.World0];
            if (world.WorldId != id.World0)
            {
                // world is free
                return false;
            }

            int shapeId = id.Index1 - 1;
            if (shapeId < 0 || (world.ShapeArray).Count <= shapeId)
            {
                return false;
            }

            Shape shape = world.ShapeArray[shapeId];
            if (shape.Id == Core.NullIndex)
            {
                // shape is free
                return false;
            }

            Debug.Assert(shape.Id == shapeId);

            return id.Revision == shape.Revision;
        }

        public static bool Chain_IsValid(ChainId id)
        {
            if (id.World0 < 0 || Core.MaxWorlds <= id.World0)
            {
                return false;
            }

            World world = Worlds[id.World0];
            if (world.WorldId != id.World0)
            {
                // world is free
                return false;
            }

            int chainId = id.Index1 - 1;
            if (chainId < 0 || (world.ChainArray).Count <= chainId)
            {
                return false;
            }

            ref ChainShape chain = ref world.ChainArray[chainId];
            if (chain.Id == Core.NullIndex)
            {
                // chain is free
                return false;
            }

            Debug.Assert(chain.Id == chainId);

            return id.Revision == chain.Revision;
        }

        public static bool Joint_IsValid(JointId id)
        {
            if (id.World0 < 0 || Core.MaxWorlds <= id.World0)
            {
                return false;
            }

            World world = Worlds[id.World0];
            if (world.WorldId != id.World0)
            {
                // world is free
                return false;
            }

            int jointId = id.Index1 - 1;
            if (jointId < 0 || (world.JointArray).Count <= jointId)
            {
                return false;
            }

            Joint joint = world.JointArray[jointId];
            if (joint.JointId == Core.NullIndex)
            {
                // joint is free
                return false;
            }

            Debug.Assert(joint.JointId == jointId);

            return id.Revision == joint.Revision;
        }

        public static void SetEnableSleeping(WorldId worldId, bool flag)
        {
            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return;
            }

            if (flag == world.EnableSleep)
            {
                return;
            }

            world.EnableSleep = flag;

            if (flag)
            {
                return;
            }

            int setCount = world.SolverSetArray.Count;
            for (int i = SolverSetType.FirstSleepingSet; i < setCount; ++i)
            {
                SolverSet set = world.SolverSetArray[i];
                if (set.Sims.Count > 0)
                {
                    SolverSet.WakeSolverSet(world, i);
                }
            }
        }

        public static void SetEnableWarmStarting(WorldId worldId, bool flag)
        {
            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return;
            }

            world.EnableWarmStarting = flag;
        }

        public static void SetEnableContinuous(WorldId worldId, bool flag)
        {
            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return;
            }

            world.EnableContinuous = flag;
        }

        public static void SetRestitutionThreshold(WorldId worldId, float value)
        {
            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return;
            }

            world.RestitutionThreshold = Math.Clamp(value, 0.0f, float.MaxValue);
        }

        public static void SetHitEventThreshold(WorldId worldId, float value)
        {
            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return;
            }

            world.HitEventThreshold = Math.Clamp(value, 0.0f, float.MaxValue);
        }

        public static void SetContactTuning(WorldId worldId, float hertz, float dampingRatio, float pushOut)
        {
            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return;
            }

            world.ContactHertz = Math.Clamp(hertz, 0.0f, float.MaxValue);
            world.ContactDampingRatio = Math.Clamp(dampingRatio, 0.0f, float.MaxValue);
            world.ContactPushOutVelocity = Math.Clamp(pushOut, 0.0f, float.MaxValue);
        }

        public static Profile GetProfile(WorldId worldId)
        {
            World world = GetWorldFromId(worldId);
            return world.Profile;
        }

        public static Counters GetCounters(WorldId worldId)
        {
            World world = GetWorldFromId(worldId);
            Counters s = new();
            s.BodyCount = world.BodyIdPool.GetIdCount();
            s.ShapeCount = world.ShapeIdPool.GetIdCount();
            s.ContactCount = world.ContactIdPool.GetIdCount();
            s.JointCount = world.JointIdPool.GetIdCount();
            s.IslandCount = world.IslandIdPool.GetIdCount();

            DynamicTree staticTree = world.BroadPhase.Trees[(int)BodyType.StaticBody];
            s.StaticTreeHeight = staticTree.GetHeight();

            DynamicTree dynamicTree = world.BroadPhase.Trees[(int)BodyType.DynamicBody];
            DynamicTree kinematicTree = world.BroadPhase.Trees[(int)BodyType.KinematicBody];
            s.TreeHeight = Math.Max(dynamicTree.GetHeight(), kinematicTree.GetHeight());

            s.StackUsed = 0;
            s.ByteCount = 0;
            s.TaskCount = world.TaskCount;

            for (int i = 0; i < Core.GraphColorCount; ++i)
            {
                s.ColorCounts[i] = world.ConstraintGraph.Colors[i].Contacts.Count + world.ConstraintGraph.Colors[i].Joints.Count;
            }

            return s;
        }

        public class WorldQueryContext
        {
            public World World;

            public OverlapResultFcn Fcn;

            public QueryFilter Filter;

            public object UserContext;

            public WorldQueryContext(World world, OverlapResultFcn fcn, QueryFilter filter, object userContext)
            {
                World = world;
                Fcn = fcn;
                Filter = filter;
                UserContext = userContext;
            }
        }

        private static readonly TreeQueryCallbackFcn _treeQueryCallback = (proxyId, shapeId, context) =>
        {
            var worldContext = (WorldQueryContext)context;
            World world = worldContext.World;

            world.ShapeArray.CheckId(shapeId);
            Shape shape = world.ShapeArray[shapeId];

            Filter shapeFilter = shape.Filter;
            QueryFilter queryFilter = worldContext.Filter;

            if ((shapeFilter.CategoryBits & queryFilter.MaskBits) == 0 || (shapeFilter.MaskBits & queryFilter.CategoryBits) == 0)
            {
                return true;
            }

            ShapeId id = (shapeId + 1, world.WorldId, shape.Revision);
            bool result = worldContext.Fcn(id, worldContext.UserContext);
            return result;
        };

        public static void OverlapAABB(WorldId worldId, in AABB aabb, in QueryFilter filter, OverlapResultFcn fcn, object context)
        {
            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return;
            }

            Debug.Assert(aabb.IsValid);

            WorldQueryContext worldContext = new(world, fcn, filter, context);

            for (int i = 0; i < Core.BodyTypeCount; ++i)
            {
                world.BroadPhase.Trees[i].Query(aabb, filter.MaskBits, _treeQueryCallback, worldContext);
            }
        }

        public class WorldOverlapContext
        {
            public World World;

            public OverlapResultFcn Fcn;

            public QueryFilter Filter;

            public DistanceProxy Proxy;

            public Transform Transform;

            public object UserContext;

            public WorldOverlapContext(World world, OverlapResultFcn fcn, QueryFilter filter, DistanceProxy proxy, Transform transform, object userContext)
            {
                World = world;
                Fcn = fcn;
                Filter = filter;
                Proxy = proxy;
                Transform = transform;
                UserContext = userContext;
            }
        }

        private static readonly TreeQueryCallbackFcn _treeOverlapCallback = (proxyId, shapeId, context) =>
        {
            var worldContext = (WorldOverlapContext)context;
            World world = worldContext.World;

            world.ShapeArray.CheckId(shapeId);
            Shape shape = world.ShapeArray[shapeId];

            Filter shapeFilter = shape.Filter;
            QueryFilter queryFilter = worldContext.Filter;

            if ((shapeFilter.CategoryBits & queryFilter.MaskBits) == 0 || (shapeFilter.MaskBits & queryFilter.CategoryBits) == 0)
            {
                return true;
            }

            Body body = Body.GetBody(world, shape.BodyId);
            Transform transform = Body.GetBodyTransformQuick(world, body);

            DistanceInput input;
            input.ProxyA = worldContext.Proxy;
            input.ProxyB = Shape.MakeShapeDistanceProxy(shape);
            input.TransformA = worldContext.Transform;
            input.TransformB = transform;
            input.UseRadii = true;

            DistanceCache cache = new();
            DistanceOutput output = DistanceFunc.ShapeDistance(ref cache, input, null, 0);

            if (output.Distance > 0.0f)
            {
                return true;
            }

            ShapeId id = (shape.Id + 1, world.WorldId, shape.Revision);
            bool result = worldContext.Fcn(id, worldContext.UserContext);
            return result;
        };

        public static void OverlapCircle(WorldId worldId, Circle circle, Transform transform, QueryFilter filter, OverlapResultFcn fcn, object context)
        {
            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return;
            }

            Debug.Assert(B2Math.Vec2_IsValid(transform.P));
            Debug.Assert(B2Math.Rot_IsValid(transform.Q));

            AABB aabb = Geometry.ComputeCircleAABB(circle, transform);
            WorldOverlapContext worldContext = new(world, fcn, filter, DistanceFunc.MakeProxy(circle.Center, circle.Radius), transform, context);

            for (int i = 0; i < Core.BodyTypeCount; ++i)
            {
                world.BroadPhase.Trees[i].Query(aabb, filter.MaskBits, _treeOverlapCallback, worldContext);
            }
        }

        public static void OverlapCapsule(WorldId worldId, Capsule capsule, Transform transform, QueryFilter filter, OverlapResultFcn fcn, object context)
        {
            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return;
            }

            Debug.Assert(B2Math.Vec2_IsValid(transform.P));
            Debug.Assert(B2Math.Rot_IsValid(transform.Q));

            AABB aabb = Geometry.ComputeCapsuleAABB(capsule, transform);
            WorldOverlapContext worldContext = new(world, fcn, filter, DistanceFunc.MakeProxy(capsule.Points, 2, capsule.Radius), transform, context);

            for (int i = 0; i < Core.BodyTypeCount; ++i)
            {
                world.BroadPhase.Trees[i].Query(aabb, filter.MaskBits, _treeOverlapCallback, worldContext);
            }
        }

        public static void OverlapPolygon(WorldId worldId, Polygon polygon, Transform transform, QueryFilter filter, OverlapResultFcn fcn, object context)
        {
            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return;
            }

            Debug.Assert(B2Math.Vec2_IsValid(transform.P));
            Debug.Assert(B2Math.Rot_IsValid(transform.Q));

            AABB aabb = Geometry.ComputePolygonAABB(polygon, transform);
            WorldOverlapContext worldContext =
                new(world, fcn, filter, DistanceFunc.MakeProxy(polygon.Vertices, polygon.Count, polygon.Radius), transform, context);

            for (int i = 0; i < Core.BodyTypeCount; ++i)
            {
                world.BroadPhase.Trees[i].Query(aabb, filter.MaskBits, _treeOverlapCallback, worldContext);
            }
        }

        private class WorldRayCastContext
        {
            public World World;

            public CastResultFcn Fcn;

            public QueryFilter Filter;

            public float Fraction;

            public object UserContext;

            public WorldRayCastContext(World world, CastResultFcn fcn, QueryFilter filter, float fraction, object userContext)
            {
                World = world;
                Fcn = fcn;
                Filter = filter;
                Fraction = fraction;
                UserContext = userContext;
            }
        }

        private static readonly TreeRayCastCallbackFcn _rayCastCallback = (ref RayCastInput input, int proxyId, int shapeId, object context) =>
        {
            var worldContext = (WorldRayCastContext)context;
            World world = worldContext.World;

            world.ShapeArray.CheckId(shapeId);
            Shape shape = world.ShapeArray[shapeId];
            Filter shapeFilter = shape.Filter;
            QueryFilter queryFilter = worldContext.Filter;

            if ((shapeFilter.CategoryBits & queryFilter.MaskBits) == 0 || (shapeFilter.MaskBits & queryFilter.CategoryBits) == 0)
            {
                return input.MaxFraction;
            }

            Body body = Body.GetBody(world, shape.BodyId);
            Transform transform = Body.GetBodyTransformQuick(world, body);
            CastOutput output = Shape.RayCastShape(input, shape, transform);

            if (output.Hit)
            {
                ShapeId id = (shapeId + 1, world.WorldId, shape.Revision);
                float fraction = worldContext.Fcn(id, output.Point, output.Normal, output.Fraction, worldContext.UserContext);
                worldContext.Fraction = fraction;
                return fraction;
            }

            return input.MaxFraction;
        };

        public static void CastRay(
            WorldId worldId,
            Vec2 origin,
            Vec2 translation,
            QueryFilter filter,
            CastResultFcn fcn,
            object context)
        {
            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return;
            }

            Debug.Assert(B2Math.Vec2_IsValid(origin));
            Debug.Assert(B2Math.Vec2_IsValid(translation));

            RayCastInput input = new(origin, translation, (float)1.0f);

            WorldRayCastContext worldContext = new(world, fcn, filter, 1.0f, context);

            for (int i = 0; i < Core.BodyTypeCount; ++i)
            {
                world.BroadPhase.Trees[i].RayCast(ref input, filter.MaskBits, _rayCastCallback, worldContext);

                if (worldContext.Fraction == 0.0f)
                {
                    return;
                }

                input.MaxFraction = worldContext.Fraction;
            }
        }

        // This callback finds the closest hit. This is the most common callback used in games.
        private static readonly CastResultFcn _rayCastClosestFcn = (shapeId, point, normal, fraction, context) =>
        {
            var rayResult = (RayResult)context;
            rayResult.ShapeId = shapeId;
            rayResult.Point = point;
            rayResult.Normal = normal;
            rayResult.Fraction = fraction;
            rayResult.Hit = true;
            return fraction;
        };

        public static RayResult CastRayClosest(WorldId worldId, Vec2 origin, Vec2 translation, QueryFilter filter)
        {
            RayResult result = B2ObjectPool<RayResult>.Shared.Get();

            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return result;
            }

            Debug.Assert(B2Math.Vec2_IsValid(origin));
            Debug.Assert(B2Math.Vec2_IsValid(translation));

            RayCastInput input = new(origin, translation, (float)1.0f);
            WorldRayCastContext worldContext = new(world, _rayCastClosestFcn, filter, 1.0f, result);

            for (int i = 0; i < Core.BodyTypeCount; ++i)
            {
                world.BroadPhase.Trees[i].RayCast(ref input, filter.MaskBits, _rayCastCallback, worldContext);

                if (worldContext.Fraction == 0.0f)
                {
                    return result;
                }

                input.MaxFraction = worldContext.Fraction;
            }

            return result;
        }

        private static float ShapeCastCallback(ref ShapeCastInput input, int proxyId, int shapeId, object context)
        {
            var worldContext = (WorldRayCastContext)context;
            World world = worldContext.World;

            world.ShapeArray.CheckId(shapeId);
            Shape shape = world.ShapeArray[shapeId];
            Filter shapeFilter = shape.Filter;
            QueryFilter queryFilter = worldContext.Filter;

            if ((shapeFilter.CategoryBits & queryFilter.MaskBits) == 0 || (shapeFilter.MaskBits & queryFilter.CategoryBits) == 0)
            {
                return input.MaxFraction;
            }

            Body body = Body.GetBody(world, shape.BodyId);
            Transform transform = Body.GetBodyTransformQuick(world, body);
            CastOutput output = Shape.ShapeCastShape(input, shape, transform);

            if (!output.Hit)
            {
                return input.MaxFraction;
            }

            ShapeId id = (shapeId + 1, world.WorldId, shape.Revision);
            float fraction = worldContext.Fcn(id, output.Point, output.Normal, output.Fraction, worldContext.UserContext);
            worldContext.Fraction = fraction;
            return fraction;
        }

        public static void CastCircle(WorldId worldId, Circle circle, Transform originTransform, Vec2 translation, QueryFilter filter, CastResultFcn fcn, object context)
        {
            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return;
            }

            Debug.Assert(B2Math.Vec2_IsValid(originTransform.P));
            Debug.Assert(B2Math.Rot_IsValid(originTransform.Q));
            Debug.Assert(B2Math.Vec2_IsValid(translation));

            ShapeCastInput input = new();
            input.Points[0] = B2Math.TransformPoint(originTransform, circle.Center);
            input.Count = 1;
            input.Radius = circle.Radius;
            input.Translation = translation;
            input.MaxFraction = 1.0f;

            WorldRayCastContext worldContext = new(world, fcn, filter, 1.0f, context);

            for (int i = 0; i < Core.BodyTypeCount; ++i)
            {
                world.BroadPhase.Trees[i].ShapeCast(ref input, filter.MaskBits, ShapeCastCallback, worldContext);

                if (worldContext.Fraction == 0.0f)
                {
                    return;
                }

                input.MaxFraction = worldContext.Fraction;
            }
        }

        public static void CastCapsule(WorldId worldId, Capsule capsule, Transform originTransform, Vec2 translation, QueryFilter filter, CastResultFcn fcn, object context)
        {
            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return;
            }

            Debug.Assert(B2Math.Vec2_IsValid(originTransform.P));
            Debug.Assert(B2Math.Rot_IsValid(originTransform.Q));
            Debug.Assert(B2Math.Vec2_IsValid(translation));

            ShapeCastInput input = new();
            input.Points[0] = B2Math.TransformPoint(originTransform, capsule.Center1);
            input.Points[1] = B2Math.TransformPoint(originTransform, capsule.Center2);
            input.Count = 2;
            input.Radius = capsule.Radius;
            input.Translation = translation;
            input.MaxFraction = 1.0f;

            WorldRayCastContext worldContext = new(world, fcn, filter, 1.0f, context);

            for (int i = 0; i < Core.BodyTypeCount; ++i)
            {
                world.BroadPhase.Trees[i].ShapeCast(ref input, filter.MaskBits, ShapeCastCallback, worldContext);

                if (worldContext.Fraction == 0.0f)
                {
                    return;
                }

                input.MaxFraction = worldContext.Fraction;
            }
        }

        public static void CastPolygon(WorldId worldId, Polygon polygon, Transform originTransform, Vec2 translation, QueryFilter filter, CastResultFcn fcn, object context)
        {
            World world = GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return;
            }

            Debug.Assert(B2Math.Vec2_IsValid(originTransform.P));
            Debug.Assert(B2Math.Rot_IsValid(originTransform.Q));
            Debug.Assert(B2Math.Vec2_IsValid(translation));

            ShapeCastInput input = new();
            for (int i = 0; i < polygon.Count; ++i)
            {
                input.Points[i] = B2Math.TransformPoint(originTransform, polygon.Vertices[i]);
            }

            input.Count = polygon.Count;
            input.Radius = polygon.Radius;
            input.Translation = translation;
            input.MaxFraction = 1.0f;

            WorldRayCastContext worldContext = new(world, fcn, filter, 1.0f, context);

            for (int i = 0; i < Core.BodyTypeCount; ++i)
            {
                world.BroadPhase.Trees[i].ShapeCast(ref input, filter.MaskBits, ShapeCastCallback, worldContext);

                if (worldContext.Fraction == 0.0f)
                {
                    return;
                }

                input.MaxFraction = worldContext.Fraction;
            }
        }

        public static void SetCustomFilterCallback(WorldId worldId, CustomFilterFcn fcn, object context)
        {
            World world = GetWorldFromId(worldId);
            world.CustomFilterFcn = fcn;
            world.CustomFilterContext = context;
        }

        public static void SetPreSolveCallback(WorldId worldId, PreSolveFcn fcn, object context)
        {
            World world = GetWorldFromId(worldId);
            world.PreSolveFcn = fcn;
            world.PreSolveContext = context;
        }

        public static void SetGravity(WorldId worldId, Vec2 gravity)
        {
            World world = GetWorldFromId(worldId);
            world.Gravity = gravity;
        }

        public static Vec2 GetGravity(WorldId worldId)
        {
            World world = GetWorldFromId(worldId);
            return world.Gravity;
        }

        public class ExplosionContext
        {
            public World World;

            public Vec2 Position;

            public float Radius;

            public float Magnitude;

            public ExplosionContext(World world, Vec2 position, float radius, float magnitude)
            {
                World = world;
                Position = position;
                Radius = radius;
                Magnitude = magnitude;
            }
        }

        private static readonly TreeQueryCallbackFcn _explosionCallback = (proxyId, shapeId, context) =>
        {
            var explosionContext = (ExplosionContext)context;
            World world = explosionContext.World;

            world.ShapeArray.CheckId(shapeId);
            Shape shape = world.ShapeArray[shapeId];

            world.BodyArray.CheckId(shape.BodyId);
            Body body = world.BodyArray[shape.BodyId];
            if (body.Type == BodyType.KinematicBody)
            {
                return true;
            }

            Body.WakeBody(world, body);

            if (body.SetIndex != SolverSetType.AwakeSet)
            {
                return true;
            }

            Transform transform = Body.GetBodyTransformQuick(world, body);

            DistanceInput input = new();
            input.ProxyA = Shape.MakeShapeDistanceProxy(shape);
            input.ProxyB = DistanceFunc.MakeProxy(explosionContext.Position, 0.0f);
            input.TransformA = transform;
            input.TransformB = Transform.Identity;
            input.UseRadii = true;

            DistanceCache cache = new();
            DistanceOutput output = DistanceFunc.ShapeDistance(ref cache, input, null, 0);

            if (output.Distance > explosionContext.Radius)
            {
                return true;
            }

            Vec2 closestPoint = output.PointA;

            if (output.Distance == 0.0f)
            {
                Vec2 localCentroid = Shape.GetShapeCentroid(shape);
                closestPoint = B2Math.TransformPoint(transform, localCentroid);
            }

            float falloff = 0.4f;
            float perimeter = shape.GetShapePerimeter();
            float magnitude = explosionContext.Magnitude * perimeter * (1.0f - falloff * output.Distance / explosionContext.Radius);

            Vec2 direction = (closestPoint - explosionContext.Position).Normalize;
            Vec2 impulse = B2Math.MulSV(magnitude, direction);

            int localIndex = body.LocalIndex;
            SolverSet set = world.SolverSetArray[SolverSetType.AwakeSet];
            Debug.Assert(0 <= localIndex && localIndex < set.States.Count);
            ref BodyState state = ref set.States.Data[localIndex];
            BodySim bodySim = set.Sims.Data[localIndex];
            state.LinearVelocity = B2Math.MulAdd(state.LinearVelocity, bodySim.InvMass, impulse);
            state.AngularVelocity += bodySim.InvInertia * B2Math.Cross(closestPoint - bodySim.Center, impulse);

            return true;
        };

        public static void Explode(WorldId worldId, Vec2 position, float radius, float magnitude)
        {
            Debug.Assert(position.IsValid());
            Debug.Assert(B2Math.IsValid(radius) && radius > 0.0f);
            Debug.Assert(B2Math.IsValid(magnitude));

            World world = World.GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return;
            }

            ExplosionContext explosionContext = new ExplosionContext(world, position, radius, magnitude);

            AABB aabb;
            aabb.LowerBound.X = position.X - radius;
            aabb.LowerBound.Y = position.Y - radius;
            aabb.UpperBound.X = position.X + radius;
            aabb.UpperBound.Y = position.Y + radius;

            world.BroadPhase.Trees[(int)BodyType.DynamicBody].Query(aabb, Core.DefaultMaskBits, _explosionCallback, explosionContext);
        }

        #endregion

        public Shape GetShape(ShapeId shapeId)
        {
            int id = shapeId.Index1 - 1;
            ShapeArray.CheckIdAndRevision(id, shapeId.Revision);
            Shape shape = ShapeArray[id];
            return shape;
        }

        public ref ChainShape GetChainShape(ChainId chainId)
        {
            int id = chainId.Index1 - 1;
            ChainArray.CheckIdAndRevision(id, chainId.Revision);
            ref ChainShape chain = ref ChainArray[id];
            return ref chain;
        }
    }
}