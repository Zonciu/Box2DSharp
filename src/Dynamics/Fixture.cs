using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics
{
    /// A fixture is used to attach a shape to a body for collision detection. A fixture
    /// inherits its transform from its parent. Fixtures hold additional non-geometric data
    /// such as friction, collision filters, etc.
    /// Fixtures are created via b2Body::CreateFixture.
    /// @warning you cannot reuse fixtures.
    public class Fixture
    {
        public Body Body;

        public float Density;

        public Filter Filter;

        public float Friction;

        public bool IsSensor
        {
            get => _isSensor;
            set
            {
                if (_isSensor != value)
                {
                    Body.IsAwake = true;
                    _isSensor    = value;
                }
            }
        }

        private bool _isSensor;

        public FixtureProxy[] Proxies;

        public int ProxyCount;

        public float Restitution;

        private Shape _shape;

        public object UserData;

        /// We need separation create/destroy functions from the constructor/destructor because
        /// the destructor cannot access the allocator (no destructor arguments allowed by C++).
        /// Todo use Object Pool
        internal static Fixture Create(Body body, FixtureDef def)
        {
            var childCount = def.shape.GetChildCount();
            var fixture = new Fixture
            {
                UserData    = def.userData,
                Friction    = def.friction,
                Restitution = def.restitution,
                Body        = body,
                Filter      = def.filter,
                IsSensor    = def.isSensor,
                _shape      = def.shape.Clone(),
                ProxyCount  = 0,
                Density     = def.density,

                // Reserve proxy space
                Proxies = new FixtureProxy[childCount]
            };
            for (var i = 0; i < childCount; ++i)
            {
                fixture.Proxies[i] = new FixtureProxy();
            }

            return fixture;
        }

        internal void Destroy()
        {
            // The proxies must be destroyed before calling this.
            Debug.Assert(ProxyCount == 0);

            // Free the proxy array.
            Proxies = null;
            _shape  = null;
        }

        // These support body activation/deactivation.
        internal void CreateProxies(in BroadPhase broadPhase, in Transform xf)
        {
            Debug.Assert(ProxyCount == 0);

            // Create proxies in the broad-phase.
            ProxyCount = _shape.GetChildCount();

            for (var i = 0; i < ProxyCount; ++i)
            {
                var proxy = Proxies[i];
                _shape.ComputeAABB(out proxy.AABB, xf, i);
                proxy.Fixture    = this;
                proxy.ChildIndex = i;
                proxy.ProxyId    = broadPhase.CreateProxy(proxy.AABB, proxy);
            }
        }

        internal void DestroyProxies(in BroadPhase broadPhase)
        {
            // Destroy proxies in the broad-phase.
            for (var i = 0; i < ProxyCount; ++i)
            {
                var proxy = Proxies[i];
                broadPhase.DestroyProxy(proxy.ProxyId);
                proxy.ProxyId = BroadPhase.NullProxy;
            }

            ProxyCount = 0;
        }

        /// Get the type of the child shape. You can use this to down cast to the concrete shape.
        /// @return the shape type.
        public ShapeType GetShapeType()
        {
            return _shape.ShapeType;
        }

        /// Get the child shape. You can modify the child shape, however you should not change the
        /// number of vertices because this will crash some collision caching mechanisms.
        /// Manipulating the shape may lead to non-physical behavior.
        public Shape GetShape()
        {
            return _shape;
        }

        /// Set the contact filtering data. This will not update contacts until the next time
        /// step when either parent body is active and awake.
        /// This automatically calls Refilter.
        public void SetFilterData(Filter filter)
        {
            Filter = filter;

            Refilter();
        }

        /// Get the contact filtering data.
        public ref readonly Filter GetFilterData()
        {
            return ref Filter;
        }

        /// Call this if you want to establish collision that was previously disabled by b2ContactFilter::ShouldCollide.
        public void Refilter()
        {
            if (Body == null)
            {
                return;
            }

            // Flag associated contacts for filtering.

            foreach (var edge in Body._contactList)
            {
                var contact = edge.Contact;
                if (contact.FixtureA == this || contact.FixtureB == this)
                {
                    contact.FlagForFiltering();
                }
            }

            if (Body._world == null)
            {
                return;
            }

            // Touch each proxy so that new pairs may be created
            var broadPhase = Body._world.ContactManager.BroadPhase;
            for (var i = 0; i < ProxyCount; ++i)
            {
                broadPhase.TouchProxy(Proxies[i].ProxyId);
            }
        }

        /// Get the parent body of this fixture. This is null if the fixture is not attached.
        /// @return the parent body.
        public Body GetBody()
        {
            return Body;
        }

        /// Get the user data that was assigned in the fixture definition. Use this to
        /// store your application specific data.
        public object GetUserData()
        {
            return UserData;
        }

        /// Set the user data. Use this to store your application specific data.
        public void SetUserData(object data)
        {
            UserData = data;
        }

        /// Test a point for containment in this fixture.
        /// @param p a point in world coordinates.
        public bool TestPoint(in Vector2 p)
        {
            return _shape.TestPoint(Body.GetTransform(), p);
        }

        /// Cast a ray against this shape.
        /// @param output the ray-cast results.
        /// @param input the ray-cast input parameters.
        public bool RayCast(out RayCastOutput output, in RayCastInput input, int childIndex)
        {
            return _shape.RayCast(out output, input, Body.GetTransform(), childIndex);
        }

        /// Get the mass data for this fixture. The mass data is based on the density and
        /// the shape. The rotational inertia is about the shape's origin. This operation
        /// may be expensive.
        public void GetMassData(out MassData massData)
        {
            _shape.ComputeMass(out massData, Density);
        }

        /// Set the density of this fixture. This will _not_ automatically adjust the mass
        /// of the body. You must call b2Body::ResetMassData to update the body's mass.
        public void SetDensity(float density)
        {
            Debug.Assert(density.IsValid() && density >= 0.0f);
            Density = density;
        }

        /// Get the density of this fixture.
        public float GetDensity()
        {
            return Density;
        }

        /// Get the coefficient of friction.
        public float GetFriction()
        {
            return Friction;
        }

        /// Set the coefficient of friction. This will _not_ change the friction of
        /// existing contacts.
        public void SetFriction(float friction)
        {
            Friction = friction;
        }

        /// Get the coefficient of restitution.
        public float GetRestitution()
        {
            return Restitution;
        }

        /// Set the coefficient of restitution. This will _not_ change the restitution of
        /// existing contacts.
        public void SetRestitution(float restitution)
        {
            Restitution = restitution;
        }

        /// Get the fixture's AABB. This AABB may be enlarge and/or stale.
        /// If you need a more accurate AABB, compute it using the shape and
        /// the body transform.
        public AABB GetAABB(int childIndex)
        {
            Debug.Assert(0 <= childIndex && childIndex < ProxyCount);
            return Proxies[childIndex].AABB;
        }

        /// Dump this fixture to the log file.
        public void Dump(int bodyIndex)
        {
            Logger.Log("    b2FixtureDef fd;");
            Logger.Log($"    fd.friction = {Friction};");
            Logger.Log($"    fd.restitution = {Restitution};");
            Logger.Log($"    fd.density = {Density};");
            Logger.Log($"    fd.isSensor = bool({IsSensor});");
            Logger.Log($"    fd.filter.categoryBits = uint16({Filter.categoryBits});");
            Logger.Log($"    fd.filter.maskBits = uint16({Filter.maskBits});");
            Logger.Log($"    fd.filter.groupIndex = int16({Filter.groupIndex});");

            switch (_shape)
            {
            case CircleShape s:
            {
                Logger.Log("    b2CircleShape shape;");
                Logger.Log($"    shape.m_radius = {s.Radius};");
                Logger.Log($"    shape.m_p.Set({s.Position.X}, {s.Position.Y});");
            }
                break;

            case EdgeShape s:
            {
                Logger.Log("    b2EdgeShape shape;");
                Logger.Log($"    shape.m_radius = {s.Radius};");
                Logger.Log($"    shape.m_vertex0.Set({s.Vertex0.X}, {s.Vertex0.Y});");
                Logger.Log($"    shape.m_vertex1.Set({s.Vertex1.X}, {s.Vertex1.Y});");
                Logger.Log($"    shape.m_vertex2.Set({s.Vertex2.X}, {s.Vertex2.Y});");
                Logger.Log($"    shape.m_vertex3.Set({s.Vertex3.X}, {s.Vertex3.Y});");
                Logger.Log($"    shape.m_hasVertex0 = bool({s.HasVertex0});");
                Logger.Log($"    shape.m_hasVertex3 = bool({s.HasVertex3});");
            }
                break;

            case PolygonShape s:
            {
                Logger.Log("    b2PolygonShape shape;");
                Logger.Log($"    b2Vec2 vs[{Settings.MaxPolygonVertices}];");
                for (var i = 0; i < s.Count; ++i)
                {
                    Logger.Log($"    vs[{i}].Set({s.Vertices[i]}, {s.Vertices[i].Y});");
                }

                Logger.Log($"    shape.Set(vs, {s.Count});");
            }
                break;

            case ChainShape s:
            {
                Logger.Log("    b2ChainShape shape;");
                Logger.Log($"    b2Vec2 vs[{s.Count}];");
                for (var i = 0; i < s.Count; ++i)
                {
                    Logger.Log($"    vs[{i}].Set({s.Vertices[i]}, {s.Vertices[i].Y});");
                }

                Logger.Log($"    shape.CreateChain(vs, {s.Count});");
                Logger.Log($"    shape.m_prevVertex.Set({s.m_prevVertex.X}, {s.m_prevVertex.Y});");
                Logger.Log($"    shape.m_nextVertex.Set({s.m_nextVertex.X}, {s.m_nextVertex.Y});");
                Logger.Log($"    shape.m_hasPrevVertex = bool({s.m_hasPrevVertex});");
                Logger.Log($"    shape.m_hasNextVertex = bool({s.m_hasNextVertex});");
            }
                break;

            default:
                return;
            }

            Logger.Log("");
            Logger.Log("    fd.shape = &shape;");
            Logger.Log("");
            Logger.Log($"    bodies[{bodyIndex}].CreateFixture(&fd);");
        }

        /// <summary>
        /// 同步粗检测AABB形状
        /// </summary>
        /// <param name="broadPhase"></param>
        /// <param name="transform1"></param>
        /// <param name="transform2"></param>
        internal void Synchronize(in BroadPhase broadPhase, in Transform transform1, in Transform transform2)
        {
            if (ProxyCount == 0)
            {
                return;
            }

            for (var i = 0; i < ProxyCount; ++i)
            {
                var proxy = Proxies[i];

                // Compute an AABB that covers the swept shape (may miss some rotation effect).

                _shape.ComputeAABB(out var aabb1, transform1, proxy.ChildIndex);
                _shape.ComputeAABB(out var aabb2, transform2, proxy.ChildIndex);

                proxy.AABB.Combine(aabb1, aabb2);

                var displacement = transform2.Position - transform1.Position;

                broadPhase.MoveProxy(proxy.ProxyId, proxy.AABB, displacement);
            }
        }
    }

    /// This holds contact filtering data.
    public struct Filter
    {
        /// The collision category bits. Normally you would just set one bit.
        public ushort categoryBits;

        /// The collision mask bits. This states the categories that this
        /// shape would accept for collision.
        public ushort maskBits;

        /// Collision groups allow a certain group of objects to never collide (negative)
        /// or always collide (positive). Zero means no collision group. Non-zero group
        /// filtering always wins against the mask bits.
        public ushort groupIndex;

        public static Filter Default = new Filter
        {
            categoryBits = 0x0001,
            maskBits     = 0xFFFF,
            groupIndex   = 0
        };
    }

    /// A fixture definition is used to create a fixture. This class defines an
    /// abstract fixture definition. You can reuse fixture definitions safely.
    public class FixtureDef
    {
        /// The density, usually in kg/m^2.
        public float density;

        /// Contact filtering data.
        public Filter filter;

        /// The friction coefficient, usually in the range [0,1].
        public float friction;

        /// A sensor shape collects contact information but never generates a collision
        /// response.
        public bool isSensor;

        /// The restitution (elasticity) usually in the range [0,1].
        public float restitution;

        /// The shape, this must be set. The shape will be cloned, so you
        /// can create the shape on the stack.
        public Shape shape;

        /// Use this to store application specific fixture data.
        public object userData;

        /// The constructor sets the default fixture definition values.
        public FixtureDef()
        {
            shape       = null;
            userData    = null;
            friction    = 0.2f;
            restitution = 0.0f;
            density     = 0.0f;
            isSensor    = false;
            filter      = Filter.Default;
        }
    }

    /// <summary>
    /// This proxy is used internally to connect fixtures to the broad-phase.
    /// 夹具代理,用于夹具和粗检测之间关联
    /// </summary>
    public class FixtureProxy
    {
        public AABB AABB;

        public int ChildIndex;

        public Fixture Fixture;

        public int ProxyId = BroadPhase.NullProxy;
    }
}