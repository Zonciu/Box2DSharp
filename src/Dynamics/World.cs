using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Numerics;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics.Contacts;
using Box2DSharp.Dynamics.Joints;

namespace Box2DSharp.Dynamics
{
    public class World
    {
        /// <summary>
        /// 新增夹具
        /// </summary>
        internal bool HasNewFixture { get; set; }

        /// <summary>
        /// 锁定世界
        /// </summary>
        internal bool IsLocked { get; set; }

        /// <summary>
        /// 清除受力
        /// </summary>
        public bool IsAutoClearForces { get; set; }

        /// <summary>
        /// 世界是否允许休眠
        /// </summary>
        public bool AllowSleep
        {
            get => _allowSleep;
            set
            {
                if (_allowSleep == value)
                {
                    return;
                }

                _allowSleep = value;
                if (_allowSleep == false)
                {
                    foreach (var b in BodyList)
                    {
                        b.IsAwake = true;
                    }
                }
            }
        }

        private bool _allowSleep;

        /// <summary>
        /// 是否启用连续碰撞
        /// </summary>
        public bool ContinuousPhysics { get; set; }

        /// <summary>
        /// 解构监听
        /// </summary>
        private IDestructionListener _destructionListener;

        /// <summary>
        /// 调试绘制
        /// </summary>
        private IDrawer _drawer;

        /// <summary>
        /// 重力常数
        /// </summary>
        private Vector2 _gravity;

        /// <summary>
        /// This is used to compute the time step ratio to
        /// support a variable time step.
        /// 时间步倍率
        /// </summary>
        private float _invDt0;

        /// <summary>
        /// 时间步完成
        /// </summary>
        private bool _stepComplete;

        /// <summary>
        /// 子步进
        /// </summary>
        private bool _subStepping;

        /// <summary>
        /// These are for debugging the solver.
        /// 热启动,用于调试求解器
        /// </summary>
        private bool _warmStarting;

        /// <summary>
        /// 性能查看
        /// </summary>
        public Profile Profile;

        public World() : this(new Vector2(0, -10))
        { }

        public World(in Vector2 gravity)
        {
            _gravity = gravity;

            _warmStarting = true;
            ContinuousPhysics = true;
            _subStepping = false;

            _stepComplete = true;

            AllowSleep = true;
            IsAutoClearForces = true;
            _invDt0 = 0.0f;
        }

        ~World()
        {
            // Some shapes allocate using b2Alloc.
            var b = BodyList.First;
            while (b != null)
            {
                var bNext = b.Next;

                foreach (var t in b.Value.FixtureList)
                {
                    t.ProxyCount = 0;
                    t.Destroy();
                }

                b = bNext;
            }
        }

        /// <summary>
        /// 接触点管理器
        /// </summary>
        public ContactManager ContactManager { get; } = new ContactManager();

        /// <summary>
        /// 物体链表
        /// </summary>
        public LinkedList<Body> BodyList { get; } = new LinkedList<Body>();

        /// <summary>
        /// 关节链表
        /// </summary>
        public LinkedList<Joint> JointList { get; } = new LinkedList<Joint>();

        /// Get the number of broad-phase proxies.
        public int ProxyCount => ContactManager.BroadPhase.GetProxyCount();

        /// Get the number of bodies.
        public int GetBodyCount => BodyList.Count;

        /// Get the number of joints.
        public int GetJointCount => JointList.Count;

        /// Get the number of contacts (each may have 0 or more contact points).
        public int GetContactCount => ContactManager.ContactList.Count;

        /// Get the height of the dynamic tree.
        public int GetTreeHeight => ContactManager.BroadPhase.GetTreeHeight();

        /// Get the balance of the dynamic tree.
        public int GetTreeBalance => ContactManager.BroadPhase.GetTreeBalance();

        /// Get the quality metric of the dynamic tree. The smaller the better.
        /// The minimum is 1.
        public float GetTreeQuality => ContactManager.BroadPhase.GetTreeQuality();

        /// <summary>
        /// Register a destruction listener. The listener is owned by you and must
        /// remain in scope.
        /// 设置解构监听器
        /// </summary>
        /// <param name="listener"></param>
        public void SetDestructionListener(IDestructionListener listener)
        {
            _destructionListener = listener;
        }

        /// <summary>
        /// Register a contact filter to provide specific control over collision.
        /// Otherwise the default filter is used (b2_defaultFilter). The listener is
        /// owned by you and must remain in scope.
        /// 注册碰撞过滤器,用于在碰撞过程中执行自定义过滤
        /// </summary>
        /// <param name="filter"></param>
        public void SetContactFilter(IContactFilter filter)
        {
            ContactManager.ContactFilter = filter;
        }

        /// <summary>
        /// Register a contact event listener. The listener is owned by you and must
        /// remain in scope.
        /// 注册接触监听器
        /// </summary>
        /// <param name="listener"></param>
        public void SetContactListener(IContactListener listener)
        {
            ContactManager.ContactListener = listener;
        }

        /// <summary>
        /// Create a rigid body given a definition. No reference to the definition
        /// is retained.
        /// @warning This function is locked during callbacks.
        /// 创建一个物体(刚体)
        /// </summary>
        /// <param name="def"></param>
        /// <returns></returns>
        public Body CreateBody(BodyDef def)
        {
            Debug.Assert(IsLocked == false);
            if (IsLocked) // 世界锁定时无法创建物体
            {
                return null;
            }

            // 创建物体并关联到本世界
            var body = new Body(def, this);

            // Add to world doubly linked list.
            // 添加物体到物体链表头部
            var bodyNode = BodyList.AddFirst(body);
            body.Node = bodyNode;
            return body;
        }

        /// <summary>
        /// Destroy a rigid body given a definition. No reference to the definition
        /// is retained. This function is locked during callbacks.
        /// @warning This automatically deletes all associated shapes and joints.
        /// @warning This function is locked during callbacks.
        /// 删除一个物体(刚体)
        /// </summary>
        /// <param name="body"></param>
        /// <returns></returns>
        public bool DestroyBody(Body body)
        {
            Debug.Assert(BodyList.Count > 0);
            Debug.Assert(IsLocked == false);
            if (IsLocked)
            {
                return false;
            }

            // Delete the attached joints.
            // 删除所有附加的关节
            foreach (var edge in body.JointList)
            {
                _destructionListener?.SayGoodbye(edge.Joint);
                DestroyJoint(edge.Joint);
            }

            // Delete the attached contacts.
            // 删除所有附加的接触点
            foreach (var edge in body.ContactList)
            {
                ContactManager.Destroy(edge.Contact);
            }

            // Delete the attached fixtures. This destroys broad-phase proxies.
            // 删除所有附加的夹具,同时会删除对应的粗检测代理
            foreach (var fixture in body.FixtureList)
            {
                _destructionListener?.SayGoodbye(fixture);
                fixture.DestroyProxies(ContactManager.BroadPhase);
            }

            // Remove world body list.
            BodyList.Remove(body.Node);
            body.Dispose();
            return true;
        }

        /// <summary>
        /// Create a joint to constrain bodies together. No reference to the definition
        /// is retained. This may cause the connected bodies to cease colliding.
        /// @warning This function is locked during callbacks.
        /// 创建关节,用于把两个物体连接在一起,在回调中不可调用
        /// </summary>
        /// <param name="def"></param>
        /// <returns></returns>
        public Joint CreateJoint(JointDef def)
        {
            Debug.Assert(IsLocked == false);
            if (IsLocked)
            {
                return null;
            }

            var j = Joint.Create(def);

            // Connect to the world list.
            // 添加到关节列表头部
            var jointNode = JointList.AddFirst(j);
            j.Node = jointNode;

            // Connect to the bodies' doubly linked lists.
            // 连接到物体的双向链表中
            j.EdgeA.Joint = j;
            j.EdgeA.Other = j.BodyB;
            j.EdgeA.Node = j.BodyA.JointList.AddFirst(j.EdgeA);

            j.EdgeB.Joint = j;
            j.EdgeB.Other = j.BodyA;
            j.EdgeB.Node = j.BodyB.JointList.AddFirst(j.EdgeB);

            var bodyA = def.BodyA;
            var bodyB = def.BodyB;

            // If the joint prevents collisions, then flag any contacts for filtering.
            if (def.CollideConnected == false)
            {
                foreach (var edge in bodyB.ContactList)
                {
                    if (edge.Other == bodyA)
                    {
                        // Flag the contact for filtering at the next time step (where either
                        // body is awake).
                        edge.Contact.FlagForFiltering();
                    }
                }
            }

            // Note: creating a joint doesn't wake the bodies.

            return j;
        }

        /// Destroy a joint. This may cause the connected bodies to begin colliding.
        /// @warning This function is locked during callbacks.
        public void DestroyJoint(Joint joint)
        {
            Debug.Assert(IsLocked == false);
            if (IsLocked)
            {
                return;
            }

            var collideConnected = joint.CollideConnected;

            // Remove from the doubly linked list.
            JointList.Remove(joint.Node);

            // Disconnect from island graph.
            var bodyA = joint.BodyA;
            var bodyB = joint.BodyB;

            // Wake up connected bodies.
            bodyA.IsAwake = true;
            bodyB.IsAwake = true;

            Debug.Assert(JointList.Count > 0);

            // If the joint prevents collisions, then flag any contacts for filtering.
            if (collideConnected == false)
            {
                foreach (var edge in bodyB.ContactList)
                {
                    if (edge.Other == bodyA)
                    {
                        // Flag the contact for filtering at the next time step (where either
                        // body is awake).
                        edge.Contact.FlagForFiltering();
                    }
                }
            }
        }

        /// Take a time step. This performs collision detection, integration,
        /// and constraint solution.
        /// @param timeStep the amount of time to simulate, this should not vary.
        /// @param velocityIterations for the velocity constraint solver.
        /// @param positionIterations for the position constraint solver.
        public void Step(
            float dt,
            int velocityIterations,
            int positionIterations)
        {
            // profile 计时
            var stepTimer = Stopwatch.StartNew();

            // If new fixtures were added, we need to find the new contacts.
            // 如果存在新增夹具,则需要找到新接触点
            if (HasNewFixture)
            {
                // 寻找新接触点
                ContactManager.FindNewContacts();

                // 去除新增夹具标志
                HasNewFixture = false;
            }

            // 锁定世界
            IsLocked = true;

            // 时间间隔与迭代次数
            var step = new TimeStep
            {
                Dt = dt,
                VelocityIterations = velocityIterations,
                PositionIterations = positionIterations
            };

            // 计算时间间隔倒数
            if (dt > 0.0f)
            {
                step.InvDt = 1.0f / dt;
            }
            else
            {
                step.InvDt = 0.0f;
            }

            step.DtRatio = _invDt0 * dt;

            step.WarmStarting = _warmStarting;
            var timer = Stopwatch.StartNew();

            // Update contacts. This is where some contacts are destroyed.
            // 更新接触点
            {
                ContactManager.Collide();
                timer.Stop();
                Profile.Collide = timer.ElapsedMilliseconds;
            }

            // Integrate velocities, solve velocity constraints, and integrate positions.
            // 对速度进行积分，求解速度约束，整合位置
            if (_stepComplete && step.Dt > 0.0f)
            {
                timer.Restart();
                Solve(step);
                timer.Stop();
                Profile.Solve = timer.ElapsedMilliseconds;
            }

            // Handle TOI events.
            // 处理碰撞时间
            if (ContinuousPhysics && step.Dt > 0.0f)
            {
                timer.Restart();
                SolveTOI(step);
                timer.Stop();
                Profile.SolveToi = timer.ElapsedMilliseconds;
            }

            if (step.Dt > 0.0f)
            {
                _invDt0 = step.InvDt;
            }

            // 启用受力清理
            if (IsAutoClearForces)
            {
                ClearForces();
            }

            // 时间步完成,解锁世界
            IsLocked = false;
            stepTimer.Stop();
            Profile.Step = stepTimer.ElapsedMilliseconds;
        }

        /// Manually clear the force buffer on all bodies. By default, forces are cleared automatically
        /// after each call to Step. The default behavior is modified by calling SetAutoClearForces.
        /// The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
        /// a fixed sized time step under a variable frame-rate.
        /// When you perform sub-stepping you will disable auto clearing of forces and instead call
        /// ClearForces after all sub-steps are complete in one pass of your game loop.
        /// @see SetAutoClearForces
        public void ClearForces()
        {
            foreach (var body in BodyList)
            {
                body.Force.SetZero();
                body.Torque = 0.0f;
            }
        }

        /// Query the world for all fixtures that potentially overlap the
        /// provided AABB.
        /// @param callback a user implemented callback class.
        /// @param aabb the query box.
        public void QueryAABB(QueryCallback callback, in AABB aabb)
        {
            ContactManager.BroadPhase.Query(
                proxyId =>
                {
                    var proxy = (FixtureProxy) ContactManager
                                              .BroadPhase.GetUserData(proxyId);
                    return callback(proxy.Fixture);
                },
                aabb);
        }

        /// Ray-cast the world for all fixtures in the path of the ray. Your callback
        /// controls whether you get the closest point, any point, or n-points.
        /// The ray-cast ignores shapes that contain the starting point.
        /// @param callback a user implemented callback class.
        /// @param point1 the ray starting point
        /// @param point2 the ray ending point
        public void RayCast(RayCastCallback callback, in Vector2 point1, in Vector2 point2)
        {
            var input = new RayCastInput
            {
                MaxFraction = 1.0f,
                P1 = point1,
                P2 = point2
            };

            ContactManager.BroadPhase.RayCast(
                (in RayCastInput subInput, int proxyId) =>
                {
                    var userData = ContactManager.BroadPhase.GetUserData(proxyId);
                    var proxy = (FixtureProxy) userData;
                    var fixture = proxy.Fixture;
                    var index = proxy.ChildIndex;

                    var hit = fixture.RayCast(out var output, input, index);

                    if (hit)
                    {
                        var fraction = output.Fraction;
                        var point = (1.0f - fraction) * input.P1 + fraction * input.P2;
                        return callback(fixture, point, output.Normal, fraction);
                    }

                    return input.MaxFraction;
                },
                input);
        }

        /// Enable/disable warm starting. For testing.
        public void SetWarmStarting(bool flag)
        {
            _warmStarting = flag;
        }

        public bool GetWarmStarting()
        {
            return _warmStarting;
        }

        /// Enable/disable single stepped continuous physics. For testing.
        public void SetSubStepping(bool flag)
        {
            _subStepping = flag;
        }

        public bool GetSubStepping()
        {
            return _subStepping;
        }

        /// Change the global gravity vector.
        public void SetGravity(in Vector2 gravity)
        {
            _gravity = gravity;
        }

        /// Get the global gravity vector.
        public Vector2 GetGravity()
        {
            return _gravity;
        }

        /// Shift the world origin. Useful for large worlds.
        /// The body shift formula is: position -= newOrigin
        /// @param newOrigin the new origin with respect to the old origin
        public void ShiftOrigin(in Vector2 newOrigin)
        {
            Debug.Assert(!IsLocked);
            if (IsLocked)
            {
                return;
            }

            foreach (var b in BodyList)
            {
                b.Transform.Position -= newOrigin;
                b.Sweep.C0 -= newOrigin;
                b.Sweep.C -= newOrigin;
            }

            foreach (var j in JointList)
            {
                j.ShiftOrigin(newOrigin);
            }

            ContactManager.BroadPhase.ShiftOrigin(newOrigin);
        }

        /// <summary>
        /// Find islands, integrate and solve constraints, solve position constraints
        /// 找出岛屿,迭代求解约束,求解位置约束(岛屿用来对物理空间进行物体分组求解,提高效率)
        /// </summary>
        /// <param name="step"></param>
        private void Solve(in TimeStep step)
        {
            Profile.SolveInit = 0.0f;
            Profile.SolveVelocity = 0.0f;
            Profile.SolvePosition = 0.0f;

            // Size the island for the worst case.
            // 最坏情况岛屿容量,即全世界在同一个岛屿
            var island = new Island(
                BodyList.Count,
                ContactManager.ContactList.Count,
                JointList.Count,
                ContactManager.ContactListener);

            // Clear all the island flags.
            // 清除所有岛屿标志
            foreach (var body in BodyList)
            {
                body.UnsetFlag(BodyFlags.Island);
            }

            foreach (var contact in ContactManager.ContactList)
            {
                contact.Flags &= ~Contact.ContactFlag.IslandFlag;
            }

            foreach (var joint in JointList)
            {
                joint.IslandFlag = false;
            }

            // Build and simulate all awake islands.
            var stackSize = BodyList.Count;
            var stack = new Stack<Body>(stackSize);
            foreach (var body in BodyList)
            {
                if (body.HasFlag(BodyFlags.Island)) // 已经分配到岛屿则跳过
                {
                    continue;
                }

                if (body.IsAwake == false || body.IsActive == false) // 跳过休眠物体
                {
                    continue;
                }

                // The seed can be dynamic or kinematic.
                if (body.BodyType == BodyType.StaticBody) // 跳过静态物体
                {
                    continue;
                }

                // Reset island and stack.
                island.Clear();

                //var stackCount = 0;
                stack.Push(body);

                //stackCount++;
                body.SetFlag(BodyFlags.Island);

                // Perform a depth first search (DFS) on the constraint graph.
                while (stack.Count > 0)
                {
                    // Grab the next body off the stack and add it to the island.
                    //--stackCount;
                    var b = stack.Pop();
                    Debug.Assert(b.IsActive);
                    island.Add(b);

                    // Make sure the body is awake (without resetting sleep timer).
                    b.SetFlag(BodyFlags.IsAwake);

                    // To keep islands as small as possible, we don't
                    // propagate islands across static bodies.
                    if (b.BodyType == BodyType.StaticBody)
                    {
                        continue;
                    }

                    // Search all contacts connected to this body.
                    // 查找该物体所有接触点
                    foreach (var contactEdge in b.ContactList)
                    {
                        var contact = contactEdge.Contact;

                        // Has this contact already been added to an island?
                        // 接触点已经标记岛屿,跳过
                        if (contact.Flags.HasFlag(Contact.ContactFlag.IslandFlag))
                        {
                            continue;
                        }

                        // Is this contact solid and touching?
                        // 接触点未启用或未接触,跳过
                        if (contact.IsEnabled() == false || contact.IsTouching() == false)
                        {
                            continue;
                        }

                        // Skip sensors.
                        // 跳过传感器
                        if (contact.FixtureA.IsSensor || contact.FixtureB.IsSensor)
                        {
                            continue;
                        }

                        // 将该接触点添加到岛屿中,并添加岛屿标志
                        island.Add(contact);
                        contact.Flags |= Contact.ContactFlag.IslandFlag;

                        var other = contactEdge.Other;

                        // Was the other body already added to this island?
                        // 如果接触边缘的另一个物体已经添加到岛屿则跳过
                        if (other.HasFlag(BodyFlags.Island))
                        {
                            continue;
                        }

                        // 否则将另一边的物体也添加到岛屿
                        //Debug.Assert(stackCount < stackSize);
                        stack.Push(other);
                        other.SetFlag(BodyFlags.Island);
                    }

                    // Search all joints connect to this body.
                    // 将该物体的关节所关联的物体也加入到岛屿中
                    foreach (var je in b.JointList)
                    {
                        if (je.Joint.IslandFlag)
                        {
                            continue;
                        }

                        var other = je.Other;

                        // Don't simulate joints connected to inactive bodies.
                        // 跳过闲置物体
                        if (other.IsActive == false)
                        {
                            continue;
                        }

                        island.Add(je.Joint);
                        je.Joint.IslandFlag = true;

                        if (other.HasFlag(BodyFlags.Island))
                        {
                            continue;
                        }

                        //Debug.Assert(stackCount < stackSize);
                        stack.Push(other);
                        other.SetFlag(BodyFlags.Island);
                    }
                }

                // 岛屿碰撞求解
                var profile = island.Solve(step, _gravity, AllowSleep);
                Profile.SolveInit += profile.SolveInit;
                Profile.SolveVelocity += profile.SolveVelocity;
                Profile.SolvePosition += profile.SolvePosition;

                // Post solve cleanup.
                for (var i = 0; i < island.BodyCount; ++i)
                {
                    // Allow static bodies to participate in other islands.
                    var b = island.Bodies[i];
                    if (b.BodyType == BodyType.StaticBody)
                    {
                        b.UnsetFlag(BodyFlags.Island);
                    }
                }
            }

            {
                var timer = Stopwatch.StartNew();

                // Synchronize fixtures, check for out of range bodies.
                foreach (var b in BodyList)
                {
                    // If a body was not in an island then it did not move.
                    if (!b.HasFlag(BodyFlags.Island))
                    {
                        continue;
                    }

                    if (b.BodyType == BodyType.StaticBody)
                    {
                        continue;
                    }

                    // Update fixtures (for broad-phase).
                    b.SynchronizeFixtures();
                }

                // Look for new contacts.
                ContactManager.FindNewContacts();
                timer.Stop();
                Profile.Broadphase = timer.ElapsedMilliseconds;
            }
        }

        /// <summary>
        /// Find TOI contacts and solve them.
        /// 求解碰撞时间
        /// </summary>
        /// <param name="step"></param>
        private void SolveTOI(in TimeStep step)
        {
            var island = new Island(
                2 * Settings.MaxToiContacts,
                Settings.MaxToiContacts,
                0,
                ContactManager.ContactListener);

            if (_stepComplete)
            {
                foreach (var b in BodyList)
                {
                    b.UnsetFlag(BodyFlags.Island);
                    b.Sweep.Alpha0 = 0.0f;
                }

                foreach (var c in ContactManager.ContactList)
                {
                    // Invalidate TOI
                    c.Flags &= ~(Contact.ContactFlag.ToiFlag | Contact.ContactFlag.IslandFlag);
                    c.ToiCount = 0;
                    c.Toi = 1.0f;
                }
            }

            // Find TOI events and solve them.
            for (;;)
            {
                // Find the first TOI.
                Contact minContact = null;
                var minAlpha = 1.0f;

                foreach (var c in ContactManager.ContactList)
                {
                    // Is this contact disabled?
                    if (c.IsEnabled() == false)
                    {
                        continue;
                    }

                    // Prevent excessive sub-stepping.
                    if (c.ToiCount > Settings.MaxSubSteps)
                    {
                        continue;
                    }

                    var alpha = 1.0f;
                    if (c.Flags.HasFlag(Contact.ContactFlag.ToiFlag))
                    {
                        // This contact has a valid cached TOI.
                        alpha = c.Toi;
                    }
                    else
                    {
                        var fA = c.GetFixtureA();
                        var fB = c.GetFixtureB();

                        // Is there a sensor?
                        // 如果接触点的夹具是传感器,不参与TOI计算,跳过
                        if (fA.IsSensor || fB.IsSensor)
                        {
                            continue;
                        }

                        var bA = fA.GetBody();
                        var bB = fB.GetBody();

                        var typeA = bA.BodyType;
                        var typeB = bB.BodyType;
                        Debug.Assert(typeA == BodyType.DynamicBody || typeB == BodyType.DynamicBody);

                        var activeA = bA.IsAwake && typeA != BodyType.StaticBody;
                        var activeB = bB.IsAwake && typeB != BodyType.StaticBody;

                        // Is at least one body active (awake and dynamic or kinematic)?
                        if (activeA == false && activeB == false)
                        {
                            continue;
                        }

                        var collideA = bA.IsBullet || typeA != BodyType.DynamicBody;
                        var collideB = bB.IsBullet || typeB != BodyType.DynamicBody;

                        // Are these two non-bullet dynamic bodies?
                        if (collideA == false && collideB == false)
                        {
                            continue;
                        }

                        // Compute the TOI for this contact.
                        // Put the sweeps onto the same time interval.
                        var alpha0 = bA.Sweep.Alpha0;

                        if (bA.Sweep.Alpha0 < bB.Sweep.Alpha0)
                        {
                            alpha0 = bB.Sweep.Alpha0;
                            bA.Sweep.Advance(alpha0);
                        }
                        else if (bB.Sweep.Alpha0 < bA.Sweep.Alpha0)
                        {
                            alpha0 = bA.Sweep.Alpha0;
                            bB.Sweep.Advance(alpha0);
                        }

                        Debug.Assert(alpha0 < 1.0f);

                        var indexA = c.GetChildIndexA();
                        var indexB = c.GetChildIndexB();

                        // Compute the time of impact in interval [0, minTOI]
                        var input = new ToiInput();
                        input.ProxyA.Set(fA.GetShape(), indexA);
                        input.ProxyB.Set(fB.GetShape(), indexB);
                        input.SweepA = bA.Sweep;
                        input.SweepB = bB.Sweep;
                        input.Tmax = 1.0f;

                        TimeOfImpact.ComputeTimeOfImpact(out var output, input);

                        // Beta is the fraction of the remaining portion of the .
                        var beta = output.Time;
                        if (output.State == ToiOutput.ToiState.Touching)
                        {
                            alpha = Math.Min(alpha0 + (1.0f - alpha0) * beta, 1.0f);
                        }
                        else
                        {
                            alpha = 1.0f;
                        }

                        c.Toi = alpha;
                        c.Flags |= Contact.ContactFlag.ToiFlag;
                    }

                    if (alpha < minAlpha)
                    {
                        // This is the minimum TOI found so far.
                        minContact = c;
                        minAlpha = alpha;
                    }
                }

                if (minContact == default || 1.0f - 10.0f * Settings.Epsilon < minAlpha)
                {
                    // No more TOI events. Done!
                    _stepComplete = true;
                    break;
                }

                // Advance the bodies to the TOI.
                var fixtureA = minContact.GetFixtureA();
                var fixtureB = minContact.GetFixtureB();
                var bodyA = fixtureA.GetBody();
                var bodyB = fixtureB.GetBody();

                var backup1 = bodyA.Sweep;
                var backup2 = bodyB.Sweep;

                bodyA.Advance(minAlpha);
                bodyB.Advance(minAlpha);

                // The TOI contact likely has some new contact points.
                minContact.Update(ContactManager.ContactListener);
                minContact.Flags &= ~Contact.ContactFlag.ToiFlag;
                ++minContact.ToiCount;

                // Is the contact solid?
                if (minContact.IsEnabled() == false || minContact.IsTouching() == false)
                {
                    // Restore the sweeps.
                    minContact.SetEnabled(false);
                    bodyA.Sweep = backup1;
                    bodyB.Sweep = backup2;
                    bodyA.SynchronizeTransform();
                    bodyB.SynchronizeTransform();
                    continue;
                }

                bodyA.IsAwake = true;
                bodyB.IsAwake = true;

                // Build the island
                island.Clear();
                island.Add(bodyA);
                island.Add(bodyB);
                island.Add(minContact);

                bodyA.SetFlag(BodyFlags.Island);
                bodyB.SetFlag(BodyFlags.Island);
                minContact.Flags |= Contact.ContactFlag.IslandFlag;

                // Get contacts on bodyA and bodyB.
                var bodies = new Body[2]
                {
                    bodyA,
                    bodyB
                };
                for (var i = 0; i < 2; ++i)
                {
                    var body = bodies[i];
                    if (body.BodyType == BodyType.DynamicBody)
                    {
                        foreach (var ce in body.ContactList)
                        {
                            if (island.BodyCount == island.Bodies.Length)
                            {
                                break;
                            }

                            if (island.ContactCount == island.Contacts.Length)
                            {
                                break;
                            }

                            var contact = ce.Contact;

                            // Has this contact already been added to the island?
                            if (contact.Flags.HasFlag(Contact.ContactFlag.IslandFlag))
                            {
                                continue;
                            }

                            // Only add static, kinematic, or bullet bodies.
                            var other = ce.Other;
                            if (other.BodyType == BodyType.DynamicBody
                             && body.IsBullet == false
                             && other.IsBullet == false)
                            {
                                continue;
                            }

                            // Skip sensors.
                            var sensorA = contact.FixtureA.IsSensor;
                            var sensorB = contact.FixtureB.IsSensor;
                            if (sensorA || sensorB)
                            {
                                continue;
                            }

                            // Tentatively advance the body to the TOI.
                            var backup = other.Sweep;
                            if (!other.HasFlag(BodyFlags.Island))
                            {
                                other.Advance(minAlpha);
                            }

                            // Update the contact points
                            contact.Update(ContactManager.ContactListener);

                            // Was the contact disabled by the user?
                            if (contact.IsEnabled() == false)
                            {
                                other.Sweep = backup;
                                other.SynchronizeTransform();
                                continue;
                            }

                            // Are there contact points?
                            if (contact.IsTouching() == false)
                            {
                                other.Sweep = backup;
                                other.SynchronizeTransform();
                                continue;
                            }

                            // Add the contact to the island
                            contact.Flags |= Contact.ContactFlag.IslandFlag;
                            island.Add(contact);

                            // Has the other body already been added to the island?
                            if (other.HasFlag(BodyFlags.Island))
                            {
                                continue;
                            }

                            // Add the other body to the island.
                            other.SetFlag(BodyFlags.Island);

                            if (other.BodyType != BodyType.StaticBody)
                            {
                                other.IsAwake = true;
                            }

                            island.Add(other);
                        }
                    }
                }

                var dt = (1.0f - minAlpha) * step.Dt;
                var subStep = new TimeStep
                {
                    Dt = dt,
                    InvDt = 1.0f / dt,
                    DtRatio = 1.0f,
                    PositionIterations = 20,
                    VelocityIterations = step.VelocityIterations,
                    WarmStarting = false
                };

                island.SolveTOI(subStep, bodyA.IslandIndex, bodyB.IslandIndex);

                // Reset island flags and synchronize broad-phase proxies.
                for (var i = 0; i < island.BodyCount; ++i)
                {
                    var body = island.Bodies[i];
                    body.UnsetFlag(BodyFlags.Island);

                    if (body.BodyType != BodyType.DynamicBody)
                    {
                        continue;
                    }

                    body.SynchronizeFixtures();

                    // Invalidate all contact TOIs on this displaced body.
                    foreach (var ce in body.ContactList)
                    {
                        ce.Contact.Flags &= ~(Contact.ContactFlag.ToiFlag | Contact.ContactFlag.IslandFlag);
                    }
                }

                // Commit fixture proxy movements to the broad-phase so that new contacts are created.
                // Also, some contacts can be destroyed.
                ContactManager.FindNewContacts();

                if (_subStepping)
                {
                    _stepComplete = false;
                    break;
                }
            }
        }

        /// Dump the world into the log file.
        /// @warning this should be called outside of a time step.
        public void Dump()
        {
            if (IsLocked)
            {
                return;
            }

            Logger.Log($"gravity = ({_gravity.X}, {_gravity.Y});");
            Logger.Log($"bodies  = {BodyList.Count};");
            Logger.Log($"joints  = {JointList.Count};");
            var i = 0;
            foreach (var b in BodyList)
            {
                b.IslandIndex = i;
                b.Dump();
                ++i;
            }

            i = 0;
            foreach (var j in JointList)
            {
                j.Index = i;
                ++i;
            }

            // First pass on joints, skip gear joints.
            foreach (var j in JointList)
            {
                if (j.JointType == JointType.GearJoint)
                {
                    continue;
                }

                Logger.Log("{");
                j.Dump();
                Logger.Log("}");
            }

            // Second pass on joints, only gear joints.
            foreach (var j in JointList)
            {
                if (j.JointType != JointType.GearJoint)
                {
                    continue;
                }

                Logger.Log("{");
                j.Dump();
                Logger.Log("}");
            }
        }

        #region Drawer

        /// <summary>
        /// Register a routine for debug drawing. The debug draw functions are called
        /// inside with b2World::DrawDebugData method. The debug draw object is owned
        /// by you and must remain in scope.
        /// 调试绘制,用于绘制物体的图形
        /// </summary>
        /// <param name="drawer"></param>
        public void SetDebugDrawer(IDrawer drawer)
        {
            _drawer = drawer;
        }

        /// Call this to draw shapes and other debug draw data. This is intentionally non-const.
        /// 绘制调试数据
        public void DrawDebugData()
        {
            if (_drawer == null)
            {
                return;
            }

            var inactiveColor = Color.FromArgb(128, 128, 77);
            var staticBodyColor = Color.FromArgb(127, 230, 127);
            var kinematicBodyColor = Color.FromArgb(127, 127, 230);
            var sleepColor = Color.FromArgb(153, 153, 153);
            var lastColor = Color.FromArgb(230, 179, 179);
            var flags = _drawer.Flags;

            if (flags.HasFlag(DrawFlag.DrawShape))
            {
                foreach (var b in BodyList)
                {
                    var xf = b.GetTransform();
                    var isActive = b.IsActive;
                    var isAwake = b.IsAwake;
                    foreach (var f in b.FixtureList)
                    {
                        if (isActive == false)
                        {
                            DrawShape(f, xf, inactiveColor);
                        }
                        else if (b.BodyType == BodyType.StaticBody)
                        {
                            DrawShape(f, xf, staticBodyColor);
                        }
                        else if (b.BodyType == BodyType.KinematicBody)
                        {
                            DrawShape(f, xf, kinematicBodyColor);
                        }
                        else if (isAwake == false)
                        {
                            DrawShape(f, xf, sleepColor);
                        }
                        else
                        {
                            DrawShape(f, xf, lastColor);
                        }
                    }
                }
            }

            if (flags.HasFlag(DrawFlag.DrawJoint))
            {
                foreach (var j in JointList)
                {
                    DrawJoint(j);
                }
            }

            if (flags.HasFlag(DrawFlag.DrawPair))
            {
                var color = Color.FromArgb(77, 230, 230);
                foreach (var c in ContactManager.ContactList)
                {
                    var fixtureA = c.GetFixtureA();
                    var fixtureB = c.GetFixtureB();

                    var cA = fixtureA.GetAABB(c.GetChildIndexA()).GetCenter();
                    var cB = fixtureB.GetAABB(c.GetChildIndexB()).GetCenter();

                    _drawer.DrawSegment(cA, cB, color);
                }
            }

            if (flags.HasFlag(DrawFlag.DrawAABB))
            {
                var color = Color.FromArgb(230, 77, 230);
                var bp = ContactManager.BroadPhase;

                foreach (var b in BodyList)
                {
                    if (b.IsActive == false)
                    {
                        continue;
                    }

                    foreach (var f in b.FixtureList)
                    {
                        foreach (var proxy in f.Proxies)
                        {
                            var aabb = bp.GetFatAABB(proxy.ProxyId);
                            var vs = new Vector2 [4];
                            vs[0].Set(aabb.LowerBound.X, aabb.LowerBound.Y);
                            vs[1].Set(aabb.UpperBound.X, aabb.LowerBound.Y);
                            vs[2].Set(aabb.UpperBound.X, aabb.UpperBound.Y);
                            vs[3].Set(aabb.LowerBound.X, aabb.UpperBound.Y);

                            _drawer.DrawPolygon(vs, 4, color);
                        }
                    }
                }
            }

            if (flags.HasFlag(DrawFlag.DrawCenterOfMass))
            {
                foreach (var b in BodyList)
                {
                    var xf = b.GetTransform();
                    xf.Position = b.GetWorldCenter();
                    _drawer.DrawTransform(xf);
                }
            }

            if (flags.HasFlag(DrawFlag.DrawContactPoint))
            {
                foreach (var contact in ContactManager.ContactList)
                {
                    var manifold = contact.GetManifold();
                    var worldManifold = contact.GetWorldManifold();
                    for (var i = 0; i < manifold.PointCount; i++)
                    {
                        _drawer.DrawPoint(worldManifold.points[i], 0.1f, Color.Blue);
                    }
                }
            }
        }

        /// <summary>
        /// 绘制关节
        /// </summary>
        /// <param name="joint"></param>
        private void DrawJoint(Joint joint)
        {
            var bodyA = joint.BodyA;
            var bodyB = joint.BodyB;
            var xf1 = bodyA.GetTransform();
            var xf2 = bodyB.GetTransform();
            var x1 = xf1.Position;
            var x2 = xf2.Position;
            var p1 = joint.GetAnchorA();
            var p2 = joint.GetAnchorB();

            var color = Color.FromArgb(127, 204, 204);

            switch (joint.JointType)
            {
            case JointType.DistanceJoint:
                _drawer.DrawSegment(p1, p2, color);
                break;

            case JointType.PulleyJoint:
            {
                var pulley = (PulleyJoint) joint;
                var s1 = pulley.GetGroundAnchorA();
                var s2 = pulley.GetGroundAnchorB();
                _drawer.DrawSegment(s1, p1, color);
                _drawer.DrawSegment(s2, p2, color);
                _drawer.DrawSegment(s1, s2, color);
            }
                break;

            case JointType.MouseJoint:
            {
                var c = Color.FromArgb(0, 255, 0);
                _drawer.DrawPoint(p1, 4.0f, c);
                _drawer.DrawPoint(p2, 4.0f, c);

                c = Color.FromArgb(204, 204, 204);
                _drawer.DrawSegment(p1, p2, c);
            }
                break;

            default:
                _drawer.DrawSegment(x1, p1, color);
                _drawer.DrawSegment(p1, p2, color);
                _drawer.DrawSegment(x2, p2, color);
                break;
            }
        }

        /// <summary>
        /// 绘制形状
        /// </summary>
        /// <param name="fixture"></param>
        /// <param name="xf"></param>
        /// <param name="color"></param>
        private void DrawShape(Fixture fixture, in Transform xf, in Color color)
        {
            switch (fixture.GetShape())
            {
            case CircleShape circle:
            {
                var center = MathUtils.Mul(xf, circle.Position);
                var radius = circle.Radius;
                var axis = MathUtils.Mul(xf.Rotation, new Vector2(1.0f, 0.0f));

                _drawer.DrawSolidCircle(center, radius, axis, color);
            }
                break;

            case EdgeShape edge:
            {
                var v1 = MathUtils.Mul(xf, edge.Vertex1);
                var v2 = MathUtils.Mul(xf, edge.Vertex2);
                _drawer.DrawSegment(v1, v2, color);
            }
                break;

            case ChainShape chain:
            {
                var count = chain.Count;
                var vertices = chain.Vertices;

                var ghostColor = Color.FromArgb(
                    color.A,
                    (int) (0.75f * color.R),
                    (int) (0.75f * color.G),
                    (int) (0.75f * color.B));

                var v1 = MathUtils.Mul(xf, vertices[0]);
                _drawer.DrawPoint(v1, 4.0f, color);

                if (chain.HasPrevVertex)
                {
                    var vp = MathUtils.Mul(xf, chain.PrevVertex);
                    _drawer.DrawSegment(vp, v1, ghostColor);
                    _drawer.DrawCircle(vp, 0.1f, ghostColor);
                }

                for (var i = 1; i < count; ++i)
                {
                    var v2 = MathUtils.Mul(xf, vertices[i]);
                    _drawer.DrawSegment(v1, v2, color);
                    _drawer.DrawPoint(v2, 4.0f, color);
                    v1 = v2;
                }

                if (chain.HasNextVertex)
                {
                    var vn = MathUtils.Mul(xf, chain.NextVertex);
                    _drawer.DrawSegment(v1, vn, ghostColor);
                    _drawer.DrawCircle(vn, 0.1f, ghostColor);
                }
            }
                break;

            case PolygonShape poly:
            {
                var vertexCount = poly.Count;
                Debug.Assert(vertexCount <= Settings.MaxPolygonVertices);
                var vertices = new Vector2[vertexCount];

                for (var i = 0; i < vertexCount; ++i)
                {
                    vertices[i] = MathUtils.Mul(xf, poly.Vertices[i]);
                }

                _drawer.DrawSolidPolygon(vertices, vertexCount, color);
            }
                break;
            }
        }

        #endregion
    }
}