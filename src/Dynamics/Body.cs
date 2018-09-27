using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics.Contacts;
using Box2DSharp.Dynamics.Joints;

namespace Box2DSharp.Dynamics
{
    /// The body type.
    /// static: zero mass, zero velocity, may be manually moved
    /// kinematic: zero mass, non-zero velocity set by user, moved by solver
    /// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
    public enum BodyType
    {
        StaticBody = 0,

        KinematicBody,

        DynamicBody

        // TODO_ERIN
        //b2_bulletBody,
    }

    /// A body definition holds all the data needed to construct a rigid body.
    /// You can safely re-use body definitions. Shapes are added to a body after construction.
    public class BodyDef
    {
        /// Does this body start out active?
        public bool Active;

        /// Set this flag to false if this body should never fall asleep. Note that
        /// this increases CPU usage.
        public bool AllowSleep;

        /// The world angle of the body in radians.
        public float Angle;

        /// Angular damping is use to reduce the angular velocity. The damping parameter
        /// can be larger than 1.0f but the damping effect becomes sensitive to the
        /// time step when the damping parameter is large.
        /// Units are 1/time
        public float AngularDamping;

        /// The angular velocity of the body.
        public float AngularVelocity;

        /// Is this body initially awake or sleeping?
        public bool Awake;

        /// The body type: static, kinematic, or dynamic.
        /// Note: if a dynamic body would have zero mass, the mass is set to one.
        public BodyType BodyType;

        /// Is this a fast moving body that should be prevented from tunneling through
        /// other moving bodies? Note that all bodies are prevented from tunneling through
        /// kinematic and static bodies. This setting is only considered on dynamic bodies.
        /// @warning You should use this flag sparingly since it increases processing time.
        public bool Bullet;

        /// Should this body be prevented from rotating? Useful for characters.
        public bool FixedRotation;

        /// Scale the gravity applied to this body.
        public float GravityScale;

        /// Linear damping is use to reduce the linear velocity. The damping parameter
        /// can be larger than 1.0f but the damping effect becomes sensitive to the
        /// time step when the damping parameter is large.
        /// Units are 1/time
        public float LinearDamping;

        /// The linear velocity of the body's origin in world co-ordinates.
        public Vector2 LinearVelocity;

        /// The world position of the body. Avoid creating bodies at the origin
        /// since this can lead to many overlapping shapes.
        public Vector2 Position;

        /// Use this to store application specific body data.
        public object UserData;

        /// This constructor sets the body definition default values.
        public BodyDef()
        {
            UserData        = null;
            Position        = Vector2.Zero;
            Angle           = 0.0f;
            LinearVelocity  = Vector2.Zero;
            AngularVelocity = 0.0f;
            LinearDamping   = 0.0f;
            AngularDamping  = 0.0f;
            AllowSleep      = true;
            Awake           = true;
            FixedRotation   = false;
            Bullet          = false;
            BodyType        = BodyType.StaticBody;
            Active          = true;
            GravityScale    = 1.0f;
        }
    }

    /// A rigid body. These are created via b2World::CreateBody.
    public class Body : IDisposable
    {
        /// <summary>
        /// 角阻尼
        /// </summary>
        internal float _angularDamping;

        /// <summary>
        /// 角速度
        /// </summary>
        internal float _angularVelocity;

        /// <summary>
        /// 接触边缘列表
        /// </summary>
        internal LinkedList<ContactEdge> _contactList;

        /// <summary>
        /// 夹具列表
        /// </summary>
        internal List<Fixture> _fixtureList;

        /// <summary>
        /// 受力
        /// </summary>
        internal Vector2 _force;

        /// <summary>
        /// 重力系数
        /// </summary>
        internal float _gravityScale;

        /// <summary>
        /// 质心的转动惯量
        /// </summary>
        private float _inertia;

        /// <summary>
        /// 质心的转动惯量倒数
        /// </summary>
        internal float _inverseInertia;

        /// <summary>
        /// 质量倒数
        /// </summary>
        internal float _invMass;

        /// <summary>
        /// 孤岛id
        /// </summary>
        internal int _islandIndex;

        /// <summary>
        /// 关节边缘列表
        /// </summary>
        internal LinkedList<JointEdge> _jointList;

        /// <summary>
        /// 线性阻尼
        /// </summary>
        internal float _linearDamping;

        /// <summary>
        /// 线速度
        /// </summary>
        internal Vector2 _linearVelocity;

        /// <summary>
        /// 质量
        /// </summary>
        internal float _mass;

        /// <summary>
        /// 休眠时间
        /// </summary>
        internal float _sleepTime;

        /// <summary>
        /// 扫描
        /// </summary>
        internal Sweep _sweep; // the swept motion for CCD

        /// <summary>
        /// 扭矩
        /// </summary>
        internal float _torque;

        /// <summary>
        /// 物体位置
        /// </summary>
        internal Transform _transform; // the body origin transform

        /// <summary>
        /// 物体类型
        /// </summary>
        internal BodyType _type;

        /// <summary>
        /// 所属世界
        /// </summary>
        internal World _world;

        /// <summary>
        /// 物体标志
        /// </summary>
        private BodyFlags Flags;

        /// <summary>
        /// 链表节点物体
        /// </summary>
        internal LinkedListNode<Body> Node;

        internal Body(BodyDef def, World world)
        {
            Debug.Assert(def.Position.IsValid());
            Debug.Assert(def.LinearVelocity.IsValid());
            Debug.Assert(def.Angle.IsValid());
            Debug.Assert(def.AngularVelocity.IsValid());
            Debug.Assert(def.AngularDamping.IsValid() && def.AngularDamping >= 0.0f);
            Debug.Assert(def.LinearDamping.IsValid() && def.LinearDamping >= 0.0f);

            Flags = 0;

            if (def.Bullet)
            {
                Flags |= BodyFlags.IsBullet;
            }

            if (def.FixedRotation)
            {
                Flags |= BodyFlags.FixedRotation;
            }

            if (def.AllowSleep)
            {
                Flags |= BodyFlags.AutoSleep;
            }

            if (def.Awake)
            {
                Flags |= BodyFlags.IsAwake;
            }

            if (def.Active)
            {
                Flags |= BodyFlags.IsActive;
            }

            _world = world;

            _transform.Position = def.Position;
            _transform.Rotation.Set(def.Angle);

            _sweep = new Sweep
            {
                localCenter = Vector2.Zero,
                c0          = _transform.Position,
                c           = _transform.Position,
                a0          = def.Angle,
                a           = def.Angle,
                alpha0      = 0.0f
            };

            _jointList   = new LinkedList<JointEdge>();
            _contactList = new LinkedList<ContactEdge>();
            _fixtureList = new List<Fixture>();
            Node         = null;

            _linearVelocity  = def.LinearVelocity;
            _angularVelocity = def.AngularVelocity;

            _linearDamping  = def.LinearDamping;
            _angularDamping = def.AngularDamping;
            _gravityScale   = def.GravityScale;

            _force.SetZero();
            _torque = 0.0f;

            _sleepTime = 0.0f;

            _type = def.BodyType;

            if (_type == BodyType.DynamicBody)
            {
                _mass    = 1.0f;
                _invMass = 1.0f;
            }
            else
            {
                _mass    = 0.0f;
                _invMass = 0.0f;
            }

            _inertia        = 0.0f;
            _inverseInertia = 0.0f;

            UserData = def.UserData;
        }

        /// Set the linear velocity of the center of mass.
        /// @param v the new linear velocity of the center of mass.
        /// Get the linear velocity of the center of mass.
        /// @return the linear velocity of the center of mass.
        public Vector2 LinearVelocity
        {
            get => _linearVelocity;
            set
            {
                if (_type == BodyType.StaticBody) // 静态物体无加速度
                {
                    return;
                }

                if (MathUtils.Dot(value, value) > 0.0f) // 点积大于0时唤醒本物体
                {
                    IsAwake = true;
                }

                _linearVelocity = value;
            }
        }

        /// <summary>
        /// Get/Set the angular velocity.
        /// the new angular velocity in radians/second.
        /// </summary>
        public float AngularVelocity
        {
            get => _angularVelocity;
            set
            {
                if (_type == BodyType.StaticBody) // 静态物体无角速度
                {
                    return;
                }

                if (value * value > 0.0f)
                {
                    IsAwake = true;
                }

                _angularVelocity = value;
            }
        }

        /// Get the total mass of the body.
        /// @return the mass, usually in kilograms (kg).
        public float Mass => _mass;

        /// Get/Set the linear damping of the body.
        public float LinearDamping
        {
            get => _linearDamping;
            set => _linearDamping = value;
        }

        /// Get/Set the angular damping of the body.
        public float AngularDamping
        {
            get => _angularDamping;
            set => _angularDamping = value;
        }

        /// Get/Set the gravity scale of the body.
        public float GravityScale
        {
            get => _gravityScale;
            set => _gravityScale = value;
        }

        /// Set the type of this body. This may alter the mass and velocity.
        public BodyType BodyType
        {
            get => _type;
            set
            {
                Debug.Assert(_world.IsLocked == false);
                if (_world.IsLocked)
                {
                    return;
                }

                if (_type == value)
                {
                    return;
                }

                _type = value;

                ResetMassData();

                if (_type == BodyType.StaticBody)
                {
                    _linearVelocity.SetZero();
                    _angularVelocity = 0.0f;
                    _sweep.a0        = _sweep.a;
                    _sweep.c0        = _sweep.c;
                    SynchronizeFixtures();
                }

                IsAwake = true;

                _force.SetZero();
                _torque = 0.0f;

                // Delete the attached contacts.
                // 删除所有接触点

                foreach (var ce in _contactList)
                {
                    _world.ContactManager.Destroy(ce.Contact);
                }

                _contactList.Clear(); // Todo ???

                // Touch the proxies so that new contacts will be created (when appropriate)
                var broadPhase = _world.ContactManager.BroadPhase;
                foreach (var f in _fixtureList)
                {
                    var proxyCount = f.ProxyCount;
                    for (var i = 0; i < proxyCount; ++i)
                    {
                        broadPhase.TouchProxy(f.Proxies[i].ProxyId);
                    }
                }
            }
        }

        /// Should this body be treated like a bullet for continuous collision detection?
        /// Is this body treated like a bullet for continuous collision detection?
        public bool IsBullet
        {
            get => Flags.HasFlag(BodyFlags.IsBullet);
            set
            {
                if (value)
                {
                    Flags |= BodyFlags.IsBullet;
                }
                else
                {
                    Flags &= ~BodyFlags.IsBullet;
                }
            }
        }

        /// You can disable sleeping on this body. If you disable sleeping, the
        /// body will be woken.
        /// Is this body allowed to sleep
        public bool IsSleepingAllowed
        {
            get => Flags.HasFlag(BodyFlags.AutoSleep);
            set
            {
                if (value)
                {
                    Flags |= BodyFlags.AutoSleep;
                }
                else
                {
                    Flags   &= ~BodyFlags.AutoSleep;
                    IsAwake =  true;
                }
            }
        }

        /// <summary>
        /// Set the sleep state of the body. A sleeping body has very
        /// low CPU cost.
        /// @param flag set to true to wake the body, false to put it to sleep.
        /// Get the sleeping state of this body.
        /// @return true if the body is awake.
        /// </summary>
        public bool IsAwake
        {
            get => Flags.HasFlag(BodyFlags.IsAwake);
            set
            {
                if (value)
                {
                    Flags      |= BodyFlags.IsAwake;
                    _sleepTime =  0.0f;
                }
                else
                {
                    Flags      &= ~BodyFlags.IsAwake;
                    _sleepTime =  0.0f;
                    _linearVelocity.SetZero();
                    _angularVelocity = 0.0f;
                    _force.SetZero();
                    _torque = 0.0f;
                }
            }
        }

        /// <summary>
        /// Set the active state of the body. An inactive body is not
        /// simulated and cannot be collided with or woken up.
        /// If you pass a flag of true, all fixtures will be added to the
        /// broad-phase.
        /// If you pass a flag of false, all fixtures will be removed from
        /// the broad-phase and all contacts will be destroyed.
        /// Fixtures and joints are otherwise unaffected. You may continue
        /// to create/destroy fixtures and joints on inactive bodies.
        /// Fixtures on an inactive body are implicitly inactive and will
        /// not participate in collisions, ray-casts, or queries.
        /// Joints connected to an inactive body are implicitly inactive.
        /// An inactive body is still owned by a b2World object and remains
        /// in the body list.
        /// Get the active state of the body.
        /// </summary>
        public bool IsActive

        {
            get => Flags.HasFlag(BodyFlags.IsActive);
            set
            {
                Debug.Assert(_world.IsLocked == false);

                if (value == IsActive)
                {
                    return;
                }

                if (value)
                {
                    Flags |= BodyFlags.IsActive;

                    // Create all proxies.
                    // 激活时创建粗检测代理
                    var broadPhase = _world.ContactManager.BroadPhase;
                    foreach (var f in _fixtureList)
                    {
                        f.CreateProxies(broadPhase, _transform);
                    }

                    // Contacts are created the next time step.
                }
                else
                {
                    Flags &= ~BodyFlags.IsActive;

                    // Destroy all proxies.
                    // 休眠时销毁粗检测代理
                    var broadPhase = _world.ContactManager.BroadPhase;
                    foreach (var f in _fixtureList)
                    {
                        f.DestroyProxies(broadPhase);
                    }

                    // Destroy the attached contacts.
                    // 销毁接触点
                    foreach (var ce in _contactList)
                    {
                        _world.ContactManager.Destroy(ce.Contact);
                    }

                    _contactList.Clear();
                }
            }
        }

        /// Set this body to have fixed rotation. This causes the mass
        /// to be reset.
        public bool IsFixedRotation
        {
            get => Flags.HasFlag(BodyFlags.FixedRotation);
            set
            {
                // 物体已经有固定旋转,不需要设置
                if (Flags.HasFlag(BodyFlags.FixedRotation) && value)
                {
                    return;
                }

                if (value)
                {
                    Flags |= BodyFlags.FixedRotation;
                }
                else
                {
                    Flags &= ~BodyFlags.FixedRotation;
                }

                _angularVelocity = 0.0f;

                ResetMassData();
            }
        }

        /// Get the list of all fixtures attached to this body.
        public List<Fixture> FixtureList => _fixtureList;

        /// Get the list of all joints attached to this body.
        public LinkedList<JointEdge> JointList => _jointList;

        /// Get the list of all contacts attached to this body.
        /// @warning this list changes during the time step and you may
        /// miss some collisions if you don't use b2ContactListener.
        public LinkedList<ContactEdge> ContactList => _contactList;

        /// <summary>
        /// Get/Set the user data pointer that was provided in the body definition.
        /// 用户信息
        /// </summary>
        public object UserData { get; set; }

        /// Get the parent world of this body.
        public World World => _world;

        /// <inheritdoc />
        public void Dispose()
        {
            _contactList.Clear();
            _jointList.Clear();
            _fixtureList.Clear();
        }

        /// <summary>
        /// Creates a fixture and attach it to this body. Use this function if you need
        /// to set some fixture parameters, like friction. Otherwise you can create the
        /// fixture directly from a shape.
        /// If the density is non-zero, this function automatically updates the mass of the body.
        /// Contacts are not created until the next time step.
        /// @param def the fixture definition.
        /// @warning This function is locked during callbacks.
        /// 创建夹具
        /// </summary>
        /// <param name="def"></param>
        /// <returns></returns>
        public Fixture CreateFixture(FixtureDef def)
        {
            Debug.Assert(_world.IsLocked == false);
            if (_world.IsLocked)
            {
                return null;
            }

            var fixture = Fixture.Create(this, def);

            if (Flags.HasFlag(BodyFlags.IsActive))
            {
                var broadPhase = _world.ContactManager.BroadPhase;
                fixture.CreateProxies(broadPhase, _transform);
            }

            fixture.Body = this;
            _fixtureList.Add(fixture);

            // Adjust mass properties if needed.
            if (fixture.Density > 0.0f)
            {
                ResetMassData();
            }

            // Let the world know we have a new fixture. This will cause new contacts
            // to be created at the beginning of the next time step.
            // 通知世界存在新增夹具,在下一个时间步中将自动创建新夹具的接触点
            _world.HasNewFixture = true;

            return fixture;
        }

        /// Creates a fixture from a shape and attach it to this body.
        /// This is a convenience function. Use b2FixtureDef if you need to set parameters
        /// like friction, restitution, user data, or filtering.
        /// If the density is non-zero, this function automatically updates the mass of the body.
        /// @param shape the shape to be cloned.
        /// @param density the shape density (set to zero for static bodies).
        /// @warning This function is locked during callbacks.
        /// 创建夹具
        public Fixture CreateFixture(Shape shape, float density)
        {
            var def = new FixtureDef
            {
                shape   = shape,
                density = density
            };

            return CreateFixture(def);
        }

        /// Destroy a fixture. This removes the fixture from the broad-phase and
        /// destroys all contacts associated with this fixture. This will
        /// automatically adjust the mass of the body if the body is dynamic and the
        /// fixture has positive density.
        /// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
        /// @param fixture the fixture to be removed.
        /// @warning This function is locked during callbacks.
        /// 删除夹具
        public void DestroyFixture(Fixture fixture)
        {
            if (fixture == default)
            {
                return;
            }

            // 世界锁定时不能删除夹具
            Debug.Assert(_world.IsLocked == false);
            if (_world.IsLocked)
            {
                return;
            }

            // 断言夹具所属物体
            Debug.Assert(fixture.Body == this);

            // Remove the fixture from this body's singly linked list.
            Debug.Assert(_fixtureList.Count > 0);

            // You tried to remove a shape that is not attached to this body.
            // 确定该夹具存在于物体的夹具列表中
            var found = _fixtureList.Any(e => e == fixture);
            Debug.Assert(found);

            // Destroy any contacts associated with the fixture.
            // 销毁关联在夹具上的接触点
            foreach (var edge in _contactList.Where(
                e => e.Contact.FixtureA == fixture || e.Contact.FixtureB == fixture))
            {
                // This destroys the contact and removes it from
                // this body's contact list.
                _world.ContactManager.Destroy(edge.Contact);
            }

            // 如果物体处于活跃状态,销毁夹具的粗检测代理对象
            if (Flags.HasFlag(BodyFlags.IsActive))
            {
                var broadPhase = _world.ContactManager.BroadPhase;
                fixture.DestroyProxies(broadPhase);
            }

            _fixtureList.Remove(fixture);
            fixture.Body = null;
            fixture.Destroy();

            // Reset the mass data.
            // 夹具销毁后重新计算物体质量
            ResetMassData();
        }

        /// Set the position of the body's origin and rotation.
        /// Manipulating a body's transform may cause non-physical behavior.
        /// Note: contacts are updated on the next call to b2World::Step.
        /// @param position the world position of the body's local origin.
        /// @param angle the world rotation in radians.
        public void SetTransform(in Vector2 position, float angle)
        {
            Debug.Assert(_world.IsLocked == false);
            if (_world.IsLocked)
            {
                return;
            }

            _transform.Rotation.Set(angle);
            _transform.Position = position;

            _sweep.c = MathUtils.Mul(_transform, _sweep.localCenter);
            _sweep.a = angle;

            _sweep.c0 = _sweep.c;
            _sweep.a0 = angle;

            var broadPhase = _world.ContactManager.BroadPhase;
            foreach (var f in _fixtureList)
            {
                f.Synchronize(broadPhase, _transform, _transform);
            }
        }

        /// Get the body transform for the body's origin.
        /// @return the world transform of the body's origin.
        public ref readonly Transform GetTransform()
        {
            return ref _transform;
        }

        /// Get the world body origin position.
        /// @return the world position of the body's origin.
        public ref readonly Vector2 GetPosition()
        {
            return ref _transform.Position;
        }

        /// Get the angle in radians.
        /// @return the current world rotation angle in radians.
        public float GetAngle()
        {
            return _sweep.a;
        }

        /// Get the world position of the center of mass.
        public ref readonly Vector2 GetWorldCenter()
        {
            return ref _sweep.c;
        }

        /// Get the local position of the center of mass.
        public ref readonly Vector2 GetLocalCenter()
        {
            return ref _sweep.localCenter;
        }

        /// Apply a force at a world point. If the force is not
        /// applied at the center of mass, it will generate a torque and
        /// affect the angular velocity. This wakes up the body.
        /// @param force the world force vector, usually in Newtons (N).
        /// @param point the world position of the point of application.
        /// @param wake also wake up the body
        /// 在指定位置施加作用力
        public void ApplyForce(in Vector2 force, in Vector2 point, bool wake)
        {
            if (_type != BodyType.DynamicBody)
            {
                return;
            }

            if (wake && !Flags.HasFlag(BodyFlags.IsAwake))
            {
                IsAwake = true;
            }

            // Don't accumulate a force if the body is sleeping.
            if (Flags.HasFlag(BodyFlags.IsAwake))
            {
                _force  += force;
                _torque += MathUtils.Cross(point - _sweep.c, force);
            }
        }

        /// Apply a force to the center of mass. This wakes up the body.
        /// @param force the world force vector, usually in Newtons (N).
        /// @param wake also wake up the body
        /// 在质心施加作用力
        public void ApplyForceToCenter(in Vector2 force, bool wake)
        {
            if (_type != BodyType.DynamicBody)
            {
                return;
            }

            if (wake && !Flags.HasFlag(BodyFlags.IsAwake))
            {
                IsAwake = true;
            }

            // Don't accumulate a force if the body is sleeping
            if (Flags.HasFlag(BodyFlags.IsAwake))
            {
                _force += force;
            }
        }

        /// Apply a torque. This affects the angular velocity
        /// without affecting the linear velocity of the center of mass.
        /// @param torque about the z-axis (out of the screen), usually in N-m.
        /// @param wake also wake up the body
        /// 施加扭矩
        public void ApplyTorque(float torque, bool wake)
        {
            if (_type != BodyType.DynamicBody)
            {
                return;
            }

            if (wake && !Flags.HasFlag(BodyFlags.IsAwake))
            {
                IsAwake = true;
            }

            // Don't accumulate a force if the body is sleeping
            if (Flags.HasFlag(BodyFlags.IsAwake))
            {
                _torque += torque;
            }
        }

        /// Apply an impulse at a point. This immediately modifies the velocity.
        /// It also modifies the angular velocity if the point of application
        /// is not at the center of mass. This wakes up the body.
        /// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
        /// @param point the world position of the point of application.
        /// @param wake also wake up the body
        /// 施加线性冲量
        public void ApplyLinearImpulse(in Vector2 impulse, in Vector2 point, bool wake)
        {
            if (_type != BodyType.DynamicBody)
            {
                return;
            }

            if (wake && !Flags.HasFlag(BodyFlags.IsAwake))
            {
                IsAwake = true;
            }

            // Don't accumulate velocity if the body is sleeping
            if (Flags.HasFlag(BodyFlags.IsAwake))
            {
                _linearVelocity  += _invMass * impulse;
                _angularVelocity += _inverseInertia * MathUtils.Cross(point - _sweep.c, impulse);
            }
        }

        /// Apply an impulse to the center of mass. This immediately modifies the velocity.
        /// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
        /// @param wake also wake up the body
        /// 在质心施加线性冲量
        private void ApplyLinearImpulseToCenter(in Vector2 impulse, bool wake)
        {
            if (_type != BodyType.DynamicBody)
            {
                return;
            }

            if (wake && !Flags.HasFlag(BodyFlags.IsAwake))
            {
                IsAwake = true;
            }

            // Don't accumulate velocity if the body is sleeping
            if (Flags.HasFlag(BodyFlags.IsAwake))
            {
                _linearVelocity += _invMass * impulse;
            }
        }

        /// Apply an angular impulse.
        /// @param impulse the angular impulse in units of kg*m*m/s
        /// @param wake also wake up the body
        public void ApplyAngularImpulse(float impulse, bool wake)
        {
            if (_type != BodyType.DynamicBody)
            {
                return;
            }

            if (wake && !Flags.HasFlag(BodyFlags.IsAwake))
            {
                IsAwake = true;
            }

            // Don't accumulate velocity if the body is sleeping
            if ((Flags & BodyFlags.IsAwake) != 0)
            {
                _angularVelocity += _inverseInertia * impulse;
            }
        }

        /// Get the rotational inertia of the body about the local origin.
        /// @return the rotational inertia, usually in kg-m^2.
        public float GetInertia()
        {
            return _inertia + _mass * MathUtils.Dot(_sweep.localCenter, _sweep.localCenter);
        }

        /// Get the mass data of the body.
        /// @return a struct containing the mass, inertia and center of the body.
        public void GetMassData(ref MassData data)
        {
            data.Mass            = _mass;
            data.RotationInertia = _inertia + _mass * MathUtils.Dot(_sweep.localCenter, _sweep.localCenter);
            data.Center          = _sweep.localCenter;
        }

        /// Set the mass properties to override the mass properties of the fixtures.
        /// Note that this changes the center of mass position.
        /// Note that creating or destroying fixtures can also alter the mass.
        /// This function has no effect if the body isn't dynamic.
        /// @param massData the mass properties.
        public void SetMassData(in MassData massData)
        {
            Debug.Assert(_world.IsLocked == false);
            if (_world.IsLocked)
            {
                return;
            }

            if (_type != BodyType.DynamicBody)
            {
                return;
            }

            _invMass        = 0.0f;
            _inertia        = 0.0f;
            _inverseInertia = 0.0f;

            _mass = massData.Mass;
            if (_mass <= 0.0f)
            {
                _mass = 1.0f;
            }

            _invMass = 1.0f / _mass;

            if (massData.RotationInertia > 0.0f && !Flags.HasFlag(BodyFlags.FixedRotation)) // 存在转动惯量且物体可旋转
            {
                _inertia = massData.RotationInertia - _mass * MathUtils.Dot(massData.Center, massData.Center);
                Debug.Assert(_inertia > 0.0f);
                _inverseInertia = 1.0f / _inertia;
            }

            // Move center of mass.
            var oldCenter = _sweep.c;
            _sweep.localCenter = massData.Center;
            _sweep.c0          = _sweep.c = MathUtils.Mul(_transform, _sweep.localCenter);

            // Update center of mass velocity.
            _linearVelocity += MathUtils.Cross(_angularVelocity, _sweep.c - oldCenter);
        }

        /// This resets the mass properties to the sum of the mass properties of the fixtures.
        /// This normally does not need to be called unless you called SetMassData to override
        /// the mass and you later want to reset the mass.
        /// 重置质量数据
        public void ResetMassData()
        {
            // Compute mass data from shapes. Each shape has its own density.
            // 从所有形状计算质量数据,每个形状都有各自的密度
            _mass           = 0.0f;
            _invMass        = 0.0f;
            _inertia        = 0.0f;
            _inverseInertia = 0.0f;
            _sweep.localCenter.SetZero();

            // Static and kinematic bodies have zero mass.
            if (_type == BodyType.StaticBody || _type == BodyType.KinematicBody)
            {
                _sweep.c0 = _transform.Position;
                _sweep.c  = _transform.Position;
                _sweep.a0 = _sweep.a;
                return;
            }

            Debug.Assert(_type == BodyType.DynamicBody);

            // Accumulate mass over all fixtures.
            var localCenter = Vector2.Zero;
            foreach (var f in _fixtureList)
            {
                if (f.Density.Equals(0.0f))
                {
                    continue;
                }

                f.GetMassData(out var massData);
                _mass       += massData.Mass;
                localCenter += massData.Mass * massData.Center;
                _inertia    += massData.RotationInertia;
            }

            // Compute center of mass.
            if (_mass > 0.0f)
            {
                _invMass    =  1.0f / _mass;
                localCenter *= _invMass;
            }
            else
            {
                // Force all dynamic bodies to have a positive mass.
                _mass    = 1.0f;
                _invMass = 1.0f;
            }

            if (_inertia > 0.0f && !Flags.HasFlag(BodyFlags.FixedRotation)) // 存在转动惯量且物体可旋转
            {
                // Center the inertia about the center of mass.
                _inertia -= _mass * MathUtils.Dot(localCenter, localCenter);
                Debug.Assert(_inertia > 0.0f);
                _inverseInertia = 1.0f / _inertia;
            }
            else
            {
                _inertia        = 0.0f;
                _inverseInertia = 0.0f;
            }

            // Move center of mass.
            var oldCenter = _sweep.c;
            _sweep.localCenter = localCenter;
            _sweep.c0          = _sweep.c = MathUtils.Mul(_transform, _sweep.localCenter);

            // Update center of mass velocity.
            _linearVelocity += MathUtils.Cross(_angularVelocity, _sweep.c - oldCenter);
        }

        /// Get the world coordinates of a point given the local coordinates.
        /// @param localPoint a point on the body measured relative the the body's origin.
        /// @return the same point expressed in world coordinates.
        public Vector2 GetWorldPoint(in Vector2 localPoint)
        {
            return MathUtils.Mul(_transform, localPoint);
        }

        /// Get the world coordinates of a vector given the local coordinates.
        /// @param localVector a vector fixed in the body.
        /// @return the same vector expressed in world coordinates.
        public Vector2 GetWorldVector(in Vector2 localVector)
        {
            return MathUtils.Mul(_transform.Rotation, localVector);
        }

        /// Gets a local point relative to the body's origin given a world point.
        /// @param a point in world coordinates.
        /// @return the corresponding local point relative to the body's origin.
        public Vector2 GetLocalPoint(in Vector2 worldPoint)
        {
            return MathUtils.MulT(_transform, worldPoint);
        }

        /// Gets a local vector given a world vector.
        /// @param a vector in world coordinates.
        /// @return the corresponding local vector.
        public Vector2 GetLocalVector(in Vector2 worldVector)
        {
            return MathUtils.MulT(_transform.Rotation, worldVector);
        }

        /// Get the world linear velocity of a world point attached to this body.
        /// @param a point in world coordinates.
        /// @return the world velocity of a point.
        public Vector2 GetLinearVelocityFromWorldPoint(in Vector2 worldPoint)
        {
            return _linearVelocity + MathUtils.Cross(_angularVelocity, worldPoint - _sweep.c);
        }

        /// Get the world velocity of a local point.
        /// @param a point in local coordinates.
        /// @return the world velocity of a point.
        public Vector2 GetLinearVelocityFromLocalPoint(in Vector2 localPoint)
        {
            return GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
        }

        /// Dump this body to a log file
        public void Dump()
        {
            var bodyIndex = _islandIndex;

            Logger.Log("{");
            Logger.Log("  b2BodyDef bd;");
            Logger.Log($"  bd.type = b2BodyType({_type});");
            Logger.Log($"  bd.position.Set({_transform.Position.X}, {_transform.Position.Y});");
            Logger.Log($"  bd.angle = {_sweep.a};");
            Logger.Log($"  bd.linearVelocity.Set({_linearVelocity.X}, {_linearVelocity.Y});");
            Logger.Log($"  bd.angularVelocity = {_angularVelocity};");
            Logger.Log($"  bd.linearDamping = {_linearDamping};");
            Logger.Log($"  bd.angularDamping = {_angularDamping};");
            Logger.Log($"  bd.allowSleep = bool({Flags.HasFlag(BodyFlags.AutoSleep)});");
            Logger.Log($"  bd.awake = bool({Flags.HasFlag(BodyFlags.IsAwake)});");
            Logger.Log($"  bd.fixedRotation = bool({Flags.HasFlag(BodyFlags.FixedRotation)});");
            Logger.Log($"  bd.bullet = bool({Flags.HasFlag(BodyFlags.IsBullet)});");
            Logger.Log($"  bd.active = bool({Flags.HasFlag(BodyFlags.IsActive)});");
            Logger.Log($"  bd.gravityScale = {_gravityScale};");
            Logger.Log($"  bodies[{_islandIndex}] = m_world.CreateBody(&bd);");
            foreach (var f in _fixtureList)
            {
                Logger.Log("  {");
                f.Dump(bodyIndex);
                Logger.Log("  }");
            }

            Logger.Log("}");
        }

        /// <summary>
        /// 同步夹具
        /// </summary>
        internal void SynchronizeFixtures()
        {
            var xf1 = new Transform();
            xf1.Rotation.Set(_sweep.a0);
            xf1.Position = _sweep.c0 - MathUtils.Mul(xf1.Rotation, _sweep.localCenter);

            var broadPhase = _world.ContactManager.BroadPhase;
            foreach (var b2Fixture in _fixtureList)
            {
                b2Fixture.Synchronize(broadPhase, xf1, _transform);
            }
        }

        /// <summary>
        /// 同步位置
        /// </summary>
        internal void SynchronizeTransform()
        {
            _transform.Rotation.Set(_sweep.a);
            _transform.Position = _sweep.c - MathUtils.Mul(_transform.Rotation, _sweep.localCenter);
        }

        // This is used to prevent connected bodies from colliding.
        // It may lie, depending on the collideConnected flag.
        internal bool ShouldCollide(Body other)
        {
            // At least one body should be dynamic.
            if (_type != BodyType.DynamicBody && other._type != BodyType.DynamicBody)
            {
                return false;
            }

            // Does a joint prevent collision?
            foreach (var joint in _jointList)
            {
                if (joint.Other == other)
                {
                    if (joint.Joint.CollideConnected == false)
                    {
                        return false;
                    }
                }
            }

            return true;
        }

        internal void Advance(float alpha)
        {
            // Advance to the new safe time. This doesn't sync the broad-phase.
            _sweep.Advance(alpha);
            _sweep.c = _sweep.c0;
            _sweep.a = _sweep.a0;
            _transform.Rotation.Set(_sweep.a);
            _transform.Position = _sweep.c - MathUtils.Mul(_transform.Rotation, _sweep.localCenter);
        }

        public bool HasFlag(BodyFlags flag)
        {
            return (Flags & flag) != 0;
        }

        public void SetFlag(BodyFlags flag)
        {
            Flags |= flag;
        }

        public void UnsetFlag(BodyFlags flag)
        {
            Flags &= ~flag;
        }
    }
}