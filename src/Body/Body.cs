using System;
using System.Diagnostics;

namespace Box2DSharp
{
    // Body organizational details that are not used in the solver.
    public class Body
    {
        public object? UserData;

        // index of solver set stored in b2World
        // may be B2_NULL_INDEX
        public int SetIndex;

        /// <summary>
        /// body sim and state index within set, may be B2_NULL_INDEX
        /// 刚体仿真数据和状态数据在解算集内的索引，有可能是 <see cref="Core.NullIndex"/>
        /// </summary>
        public int LocalIndex;

        // [31 : contactId | 1 : edgeIndex]
        public int HeadContactKey;

        public int ContactCount;

        // todo maybe move this to the body sim
        public int HeadShapeId;

        public int ShapeCount;

        public int HeadChainId;

        // [31 : jointId | 1 : edgeIndex]
        public int HeadJointKey;

        public int JointCount;

        // All enabled dynamic and kinematic bodies are in an island.
        public int IslandId;

        // doubly-linked island list
        public int IslandPrev;

        public int IslandNext;

        public float SleepThreshold;

        public float SleepTime;

        // this is used to adjust the fellAsleep flag in the body move array
        public int BodyMoveIndex;

        public int Id;

        public BodyType Type;

        // This is monotonically advanced when a body is allocated in this slot
        // Used to check for invalid b2BodyId
        public ushort Revision;

        public bool EnableSleep;

        public bool FixedRotation;

        public bool IsSpeedCapped;

        public bool IsMarked;

        public bool AutomaticMass;

        public static Body GetBody(World world, int bodyId)
        {
            world.BodyArray.CheckIndex(bodyId);
            return world.BodyArray[bodyId];
        }

        // Get a validated body from a world using an id.
        public static Body GetBodyFullId(World world, BodyId bodyId)
        {
            Debug.Assert(bodyId.IsValid());

            // id index starts at one so that zero can represent null
            return GetBody(world, bodyId.Index1 - 1);
        }

        public static Transform GetBodyTransformQuick(World world, Body body)
        {
            world.SolverSetArray.CheckIndex(body.SetIndex);
            ref readonly SolverSet set = ref world.SolverSetArray[body.SetIndex];
            Debug.Assert(0 <= body.LocalIndex && body.LocalIndex <= set.Sims.Count);
            BodySim bodySim = set.Sims.Data[body.LocalIndex];
            return bodySim.Transform;
        }

        public static Transform GetBodyTransform(World world, int bodyId)
        {
            world.BodyArray.CheckIndex(bodyId);
            Body body = world.BodyArray[bodyId];
            return GetBodyTransformQuick(world, body);
        }

        // Create a b2BodyId from a raw id.
        public static BodyId MakeBodyId(World world, int bodyId)
        {
            world.BodyArray.CheckIndex(bodyId);
            Body body = world.BodyArray[bodyId];
            return new BodyId(bodyId + 1, world.WorldId, body.Revision)
                ;
        }

        public static BodySim GetBodySim(World world, Body body)
        {
            world.SolverSetArray.CheckIndex(body.SetIndex);
            ref readonly SolverSet set = ref world.SolverSetArray[body.SetIndex];
            Debug.Assert(0 <= body.LocalIndex && body.LocalIndex < set.Sims.Count);
            return set.Sims.Data[body.LocalIndex];
        }

        public static ref BodyState GetBodyState(World world, Body body, out bool success)
        {
            world.SolverSetArray.CheckIndex(body.SetIndex);
            if (body.SetIndex == SolverSetType.AwakeSet)
            {
                SolverSet set = world.SolverSetArray[SolverSetType.AwakeSet];
                Debug.Assert(0 <= body.LocalIndex && body.LocalIndex < set.States.Count);
                ref var state = ref set.States[body.LocalIndex];
                success = true;
                return ref state;
            }

            success = false;
            return ref BodyState.Null;
        }

        public static void CreateIslandForBody(World world, int setIndex, Body body)
        {
            Debug.Assert(body.IslandId == Core.NullIndex);
            Debug.Assert(body.IslandPrev == Core.NullIndex);
            Debug.Assert(body.IslandNext == Core.NullIndex);
            Debug.Assert(setIndex != SolverSetType.DisabledSet);

            Island island = Island.CreateIsland(world, setIndex);

            body.IslandId = island.IslandId;
            island.HeadBody = body.Id;
            island.TailBody = body.Id;
            island.BodyCount = 1;
        }

        public static void RemoveBodyFromIsland(World world, Body body)
        {
            if (body.IslandId == Core.NullIndex)
            {
                Debug.Assert(body.IslandPrev == Core.NullIndex);
                Debug.Assert(body.IslandNext == Core.NullIndex);
                return;
            }

            int islandId = body.IslandId;
            world.IslandArray.CheckIndex(islandId);
            Island island = world.IslandArray[islandId];

            // Fix the island's linked list of sims
            if (body.IslandPrev != Core.NullIndex)
            {
                Body prevBody = GetBody(world, body.IslandPrev);
                prevBody.IslandNext = body.IslandNext;
            }

            if (body.IslandNext != Core.NullIndex)
            {
                Body nextBody = GetBody(world, body.IslandNext);
                nextBody.IslandPrev = body.IslandPrev;
            }

            Debug.Assert(island.BodyCount > 0);
            island.BodyCount -= 1;
            bool islandDestroyed = false;

            if (island.HeadBody == body.Id)
            {
                island.HeadBody = body.IslandNext;

                if (island.HeadBody == Core.NullIndex)
                {
                    // Destroy empty island
                    Debug.Assert(island.TailBody == body.Id);
                    Debug.Assert(island.BodyCount == 0);
                    Debug.Assert(island.ContactCount == 0);
                    Debug.Assert(island.JointCount == 0);

                    // Free the island
                    Island.DestroyIsland(world, island.IslandId);
                    islandDestroyed = true;
                }
            }
            else if (island.TailBody == body.Id)
            {
                island.TailBody = body.IslandPrev;
            }

            if (islandDestroyed == false)
            {
                Island.ValidateIsland(world, islandId);
            }

            body.IslandId = Core.NullIndex;
            body.IslandPrev = Core.NullIndex;
            body.IslandNext = Core.NullIndex;
        }

        public static void DestroyBodyContacts(World world, Body body, bool wakeBodies)
        {
            // Destroy the attached contacts
            int edgeKey = body.HeadContactKey;
            while (edgeKey != Core.NullIndex)
            {
                int contactId = edgeKey >> 1;
                int edgeIndex = edgeKey & 1;

                ref Contact contact = ref world.ContactArray[contactId];
                edgeKey = contact.Edges[edgeIndex].NextKey;
                Contact.DestroyContact(world, contact, wakeBodies);
            }

            SolverSet.ValidateSolverSets(world);
        }

        public static BodyId CreateBody(WorldId worldId, in BodyDef def)
        {
            def.CheckDef();
            Debug.Assert(B2Math.Vec2_IsValid(def.Position));
            Debug.Assert(B2Math.Rot_IsValid(def.Rotation));
            Debug.Assert(B2Math.Vec2_IsValid(def.LinearVelocity));
            Debug.Assert(B2Math.IsValid(def.AngularVelocity));
            Debug.Assert(B2Math.IsValid(def.LinearDamping) && def.LinearDamping >= 0.0f);
            Debug.Assert(B2Math.IsValid(def.AngularDamping) && def.AngularDamping >= 0.0f);
            Debug.Assert(B2Math.IsValid(def.SleepThreshold) && def.SleepThreshold >= 0.0f);
            Debug.Assert(B2Math.IsValid(def.GravityScale));

            World world = World.GetWorldFromId(worldId);
            Debug.Assert(world.Locked == false);

            if (world.Locked)
            {
                return BodyId.NullId;
            }

            bool isAwake = (def.IsAwake || def.EnableSleep == false) && def.IsEnabled;

            // determine the solver set
            int setId;
            if (def.IsEnabled == false)
            {
                // any body type can be disabled
                setId = SolverSetType.DisabledSet;
            }
            else if (def.Type == BodyType.StaticBody)
            {
                setId = SolverSetType.StaticSet;
            }
            else if (isAwake == true)
            {
                setId = SolverSetType.AwakeSet;
            }
            else
            {
                // new set for a sleeping body in its own island
                setId = world.SolverSetIdPool.AllocId();
                if (setId == world.SolverSetArray.Count)
                {
                    world.SolverSetArray.Push(new SolverSet());
                }
                else
                {
                    Debug.Assert(world.SolverSetArray[setId].SetIndex == Core.NullIndex);
                }

                world.SolverSetArray[setId].SetIndex = setId;
            }

            Debug.Assert(0 <= setId && setId < world.SolverSetArray.Count);

            int bodyId = world.BodyIdPool.AllocId();

            SolverSet set = world.SolverSetArray[setId];

            var bodySim = set.Sims.AddBodySim();
            bodySim.Transform.P = def.Position;
            bodySim.Transform.Q = def.Rotation;
            bodySim.Center = def.Position;
            bodySim.Rotation0 = bodySim.Transform.Q;
            bodySim.Center0 = bodySim.Center;
            bodySim.LocalCenter = Vec2.Zero;
            bodySim.Force = Vec2.Zero;
            bodySim.Torque = 0.0f;
            bodySim.Mass = 0.0f;
            bodySim.InvMass = 0.0f;
            bodySim.Inertia = 0.0f;
            bodySim.InvInertia = 0.0f;
            bodySim.MinExtent = Core.Huge;
            bodySim.MaxExtent = 0.0f;
            bodySim.LinearDamping = def.LinearDamping;
            bodySim.AngularDamping = def.AngularDamping;
            bodySim.GravityScale = def.GravityScale;
            bodySim.BodyId = bodyId;
            bodySim.IsBullet = def.IsBullet;
            bodySim.AllowFastRotation = def.AllowFastRotation;
            bodySim.EnlargeAABB = false;
            bodySim.IsFast = false;
            bodySim.IsSpeedCapped = false;

            if (setId == SolverSetType.AwakeSet)
            {
                ref BodyState bodyState = ref set.States.AddBodyState();
                bodyState.LinearVelocity = def.LinearVelocity;
                bodyState.AngularVelocity = def.AngularVelocity;
                bodyState.DeltaRotation = Rot.Identity;
            }

            if (bodyId == world.BodyArray.Count)
            {
                world.BodyArray.Push(new Body());
            }
            else
            {
                Debug.Assert(world.BodyArray[bodyId].Id == Core.NullIndex);
            }

            world.BodyArray.CheckIndex(bodyId);
            Body body = world.BodyArray[bodyId];
            body.UserData = def.UserData;
            body.SetIndex = setId;
            body.LocalIndex = set.Sims.Count - 1;
            body.Revision += 1;
            body.HeadShapeId = Core.NullIndex;
            body.ShapeCount = 0;
            body.HeadChainId = Core.NullIndex;
            body.HeadContactKey = Core.NullIndex;
            body.ContactCount = 0;
            body.HeadJointKey = Core.NullIndex;
            body.JointCount = 0;
            body.IslandId = Core.NullIndex;
            body.IslandPrev = Core.NullIndex;
            body.IslandNext = Core.NullIndex;
            body.BodyMoveIndex = Core.NullIndex;
            body.Id = bodyId;
            body.SleepThreshold = def.SleepThreshold;
            body.SleepTime = 0.0f;
            body.Type = def.Type;
            body.EnableSleep = def.EnableSleep;
            body.FixedRotation = def.FixedRotation;
            body.IsSpeedCapped = false;
            body.IsMarked = false;
            body.AutomaticMass = def.AutomaticMass;

            // dynamic and kinematic bodies that are enabled need a island
            if (setId >= SolverSetType.AwakeSet)
            {
                CreateIslandForBody(world, setId, body);
            }

            SolverSet.ValidateSolverSets(world);

            BodyId id = new(bodyId + 1, world.WorldId, body.Revision);
            return id;
        }

        public static bool IsBodyAwake(World world, Body body)
        {
            return body.SetIndex == SolverSetType.AwakeSet;
        }

        public static bool WakeBody(World world, Body body)
        {
            if (body.SetIndex >= SolverSetType.FirstSleepingSet)
            {
                SolverSet.WakeSolverSet(world, body.SetIndex);
                return true;
            }

            return false;
        }

        public static void DestroyBody(BodyId bodyId)
        {
            var world = World.GetWorldLocked(bodyId.World0);

            Body body = GetBodyFullId(world, bodyId);

            // Wake bodies attached to this body, even if this body is static.
            bool wakeBodies = true;

            // Destroy the attached joints
            int edgeKey = body.HeadJointKey;
            while (edgeKey != Core.NullIndex)
            {
                int jointId = edgeKey >> 1;
                int edgeIndex = edgeKey & 1;

                Joint joint = world.JointArray[jointId];
                edgeKey = joint.Edges[edgeIndex].NextKey;

                // Careful because this modifies the list being traversed
                Joint.DestroyJointInternal(world, joint, wakeBodies);
            }

            // Destroy all contacts attached to this body.
            DestroyBodyContacts(world, body, wakeBodies);

            // Destroy the attached shapes and their broad-phase proxies.
            int shapeId = body.HeadShapeId;
            while (shapeId != Core.NullIndex)
            {
                Shape shape = world.ShapeArray[shapeId];

                Shape.DestroyShapeProxy(shape, world.BroadPhase);

                // Return shape to free list.
                world.ShapeIdPool.FreeId(shapeId);
                shape.Id = Core.NullIndex;

                shapeId = shape.NextShapeId;
            }

            // Destroy the attached chains. The associated shapes have already been destroyed above.
            int chainId = body.HeadChainId;
            while (chainId != Core.NullIndex)
            {
                ref ChainShape chain = ref world.ChainArray[chainId];

                chain.ShapeIndices = null!;

                // Return chain to free list.
                world.ChainIdPool.FreeId(chainId);
                chain.Id = Core.NullIndex;

                chainId = chain.NextChainId;
            }

            RemoveBodyFromIsland(world, body);

            // Remove body sim from solver set that owns it
            world.SolverSetArray.CheckIndex(body.SetIndex);
            SolverSet set = world.SolverSetArray[body.SetIndex];
            int movedIndex = set.Sims.RemoveBodySim(body.LocalIndex);
            if (movedIndex != Core.NullIndex)
            {
                // Fix moved body index
                BodySim movedSim = set.Sims.Data[body.LocalIndex];
                int movedId = movedSim.BodyId;
                Body movedBody = world.BodyArray[movedId];
                Debug.Assert(movedBody.LocalIndex == movedIndex);
                movedBody.LocalIndex = body.LocalIndex;
            }

            // Remove body state from awake set
            if (body.SetIndex == SolverSetType.AwakeSet)
            {
                int result = set.States.RemoveBodyState(body.LocalIndex);
                Debug.Assert(result == movedIndex);
            }

            // Free body and id (preserve body revision)
            world.BodyIdPool.FreeId(body.Id);

            body.SetIndex = Core.NullIndex;
            body.LocalIndex = Core.NullIndex;
            body.Id = Core.NullIndex;

            SolverSet.ValidateSolverSets(world);
        }

        public static int GetContactCapacity(BodyId bodyId)
        {
            var world = World.GetWorldLocked(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);

            // Conservative and fast
            return body.ContactCount;
        }

        // todo what about sensors?
        // todo sample needed
        public static int GetContactData(BodyId bodyId, ContactData[] contactData, int capacity)
        {
            var world = World.GetWorldLocked(bodyId.World0);

            Body body = GetBodyFullId(world, bodyId);

            int contactKey = body.HeadContactKey;
            int index = 0;
            while (contactKey != Core.NullIndex && index < capacity)
            {
                int contactId = contactKey >> 1;
                int edgeIndex = contactKey & 1;

                world.ContactArray.CheckIndex(contactId);
                Contact contact = world.ContactArray[contactId];

                // Is contact touching?
                if (contact.Flags.IsSet(ContactFlags.ContactTouchingFlag))
                {
                    Shape shapeA = world.ShapeArray[contact.ShapeIdA];
                    Shape shapeB = world.ShapeArray[contact.ShapeIdB];

                    contactData[index].ShapeIdA = new ShapeId(shapeA.Id + 1, bodyId.World0, shapeA.Revision);
                    contactData[index].ShapeIdB = new ShapeId(shapeB.Id + 1, bodyId.World0, shapeB.Revision);

                    ContactSim contactSim = Contact.GetContactSim(world, contact);
                    contactData[index].Manifold = contactSim.Manifold;

                    index += 1;
                }

                contactKey = contact.Edges[edgeIndex].NextKey;
            }

            Debug.Assert(index <= capacity);
            return index;
        }

        public static AABB ComputeAABB(BodyId bodyId)
        {
            var world = World.GetWorldLocked(bodyId.World0);

            Body body = GetBodyFullId(world, bodyId);
            if (body.HeadShapeId == Core.NullIndex)
            {
                Transform transform = GetBodyTransform(world, body.Id);
                return (transform.P, transform.P);
            }

            Shape shape = world.ShapeArray[body.HeadShapeId];
            AABB aabb = shape.AABB;
            while (shape.NextShapeId != Core.NullIndex)
            {
                shape = world.ShapeArray[shape.NextShapeId];
                aabb = B2Math.AABB_Union(aabb, shape.AABB);
            }

            return aabb;
        }

        public static void UpdateBodyMassData(World world, Body body)
        {
            BodySim bodySim = GetBodySim(world, body);

            // Compute mass data from shapes. Each shape has its own density.
            bodySim.Mass = 0.0f;
            bodySim.InvMass = 0.0f;
            bodySim.Inertia = 0.0f;
            bodySim.InvInertia = 0.0f;
            bodySim.LocalCenter = Vec2.Zero;
            bodySim.MinExtent = Core.Huge;
            bodySim.MaxExtent = 0.0f;

            int shapeId;

            // Static and kinematic sims have zero mass.
            if (body.Type != BodyType.DynamicBody)
            {
                bodySim.Center = bodySim.Transform.P;

                // Need extents for kinematic bodies for sleeping to work correctly.
                if (body.Type == BodyType.KinematicBody)
                {
                    shapeId = body.HeadShapeId;
                    while (shapeId != Core.NullIndex)
                    {
                        Shape s = world.ShapeArray[shapeId];

                        ShapeExtent extent = s.ComputeShapeExtent(Vec2.Zero);
                        bodySim.MinExtent = Math.Min(bodySim.MinExtent, extent.MinExtent);
                        bodySim.MaxExtent = Math.Max(bodySim.MaxExtent, extent.MaxExtent);

                        shapeId = s.NextShapeId;
                    }
                }

                return;
            }

            // Accumulate mass over all shapes.
            Vec2 localCenter = Vec2.Zero;
            shapeId = body.HeadShapeId;
            while (shapeId != Core.NullIndex)
            {
                Shape s = world.ShapeArray[shapeId];
                shapeId = s.NextShapeId;

                if (s.Density == 0.0f)
                {
                    continue;
                }

                MassData massData = s.ComputeShapeMass();
                bodySim.Mass += massData.Mass;
                localCenter = B2Math.MulAdd(localCenter, massData.Mass, massData.Center);
                bodySim.Inertia += massData.RotationalInertia;
            }

            // Compute center of mass.
            if (bodySim.Mass > 0.0f)
            {
                bodySim.InvMass = 1.0f / bodySim.Mass;
                localCenter = B2Math.MulSV(bodySim.InvMass, localCenter);
            }

            if (bodySim.Inertia > 0.0f && body.FixedRotation == false)
            {
                // Center the inertia about the center of mass.
                bodySim.Inertia -= bodySim.Mass * B2Math.Dot(localCenter, localCenter);
                Debug.Assert(bodySim.Inertia > 0.0f);
                bodySim.InvInertia = 1.0f / bodySim.Inertia;
            }
            else
            {
                bodySim.Inertia = 0.0f;
                bodySim.InvInertia = 0.0f;
            }

            // Move center of mass.
            Vec2 oldCenter = bodySim.Center;
            bodySim.LocalCenter = localCenter;
            bodySim.Center = B2Math.TransformPoint(bodySim.Transform, bodySim.LocalCenter);

            // Update center of mass velocity
            ref var state = ref GetBodyState(world, body, out var success);
            if (success)
            {
                Vec2 deltaLinear = B2Math.CrossSV(state.AngularVelocity, B2Math.Sub(bodySim.Center, oldCenter));
                state.LinearVelocity = B2Math.Add(state.LinearVelocity, deltaLinear);
            }

            // Compute body extents relative to center of mass
            shapeId = body.HeadShapeId;
            while (shapeId != Core.NullIndex)
            {
                Shape s = world.ShapeArray[shapeId];

                ShapeExtent extent = s.ComputeShapeExtent(localCenter);
                bodySim.MinExtent = Math.Min(bodySim.MinExtent, extent.MinExtent);
                bodySim.MaxExtent = Math.Max(bodySim.MaxExtent, extent.MaxExtent);

                shapeId = s.NextShapeId;
            }
        }

        public static Vec2 GetPosition(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            Transform transform = GetBodyTransformQuick(world, body);
            return transform.P;
        }

        public static Rot GetRotation(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            Transform transform = GetBodyTransformQuick(world, body);
            return transform.Q;
        }

        public static Transform GetTransform(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            return GetBodyTransformQuick(world, body);
        }

        public static Vec2 GetLocalPoint(BodyId bodyId, Vec2 worldPoint)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            Transform transform = GetBodyTransformQuick(world, body);
            return B2Math.InvTransformPoint(transform, worldPoint);
        }

        public static Vec2 GetWorldPoint(BodyId bodyId, Vec2 localPoint)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            Transform transform = GetBodyTransformQuick(world, body);
            return B2Math.TransformPoint(transform, localPoint);
        }

        public static Vec2 GetLocalVector(BodyId bodyId, Vec2 worldVector)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            Transform transform = GetBodyTransformQuick(world, body);
            return B2Math.InvRotateVector(transform.Q, worldVector);
        }

        public static Vec2 GetWorldVector(BodyId bodyId, Vec2 localVector)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            Transform transform = GetBodyTransformQuick(world, body);
            return B2Math.RotateVector(transform.Q, localVector);
        }

        public static void SetTransform(BodyId bodyId, Vec2 position, Rot rotation)
        {
            Debug.Assert(B2Math.Vec2_IsValid(position));
            Debug.Assert(B2Math.Rot_IsValid(rotation));
            Debug.Assert(bodyId.IsValid());
            World world = World.GetWorld(bodyId.World0);
            Debug.Assert(world.Locked == false);

            Body body = GetBodyFullId(world, bodyId);
            BodySim bodySim = GetBodySim(world, body);

            bodySim.Transform.P = position;
            bodySim.Transform.Q = rotation;
            bodySim.Center = B2Math.TransformPoint(bodySim.Transform, bodySim.LocalCenter);

            bodySim.Rotation0 = bodySim.Transform.Q;
            bodySim.Center0 = bodySim.Center;

            BroadPhase broadPhase = world.BroadPhase;

            Transform transform = bodySim.Transform;
            float margin = Core.b2_aabbMargin;
            float speculativeDistance = Core.SpeculativeDistance;

            int shapeId = body.HeadShapeId;
            while (shapeId != Core.NullIndex)
            {
                Shape shape = world.ShapeArray[shapeId];
                AABB aabb = shape.ComputeShapeAABB(transform);
                aabb.LowerBound.X -= speculativeDistance;
                aabb.LowerBound.Y -= speculativeDistance;
                aabb.UpperBound.X += speculativeDistance;
                aabb.UpperBound.Y += speculativeDistance;
                shape.AABB = aabb;

                if (B2Math.AABB_Contains(shape.FatAABB, aabb) == false)
                {
                    AABB fatAABB;
                    fatAABB.LowerBound.X = aabb.LowerBound.X - margin;
                    fatAABB.LowerBound.Y = aabb.LowerBound.Y - margin;
                    fatAABB.UpperBound.X = aabb.UpperBound.X + margin;
                    fatAABB.UpperBound.Y = aabb.UpperBound.Y + margin;
                    shape.FatAABB = fatAABB;

                    // They body could be disabled
                    if (shape.ProxyKey != Core.NullIndex)
                    {
                        broadPhase.MoveProxy(shape.ProxyKey, fatAABB);
                    }
                }

                shapeId = shape.NextShapeId;
            }
        }

        public static Vec2 GetLinearVelocity(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);

            ref var state = ref GetBodyState(world, body, out var success);
            if (success)
            {
                return state.LinearVelocity;
            }

            return Vec2.Zero;
        }

        public static float GetAngularVelocity(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);

            ref var state = ref GetBodyState(world, body, out var success);
            if (success)
            {
                return state.AngularVelocity;
            }

            return 0.0f;
        }

        public static void SetLinearVelocity(BodyId bodyId, Vec2 linearVelocity)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);

            if (B2Math.LengthSquared(linearVelocity) > 0.0f)
            {
                WakeBody(world, body);
            }

            ref var state = ref GetBodyState(world, body, out var success);
            if (success)
            {
                state.LinearVelocity = linearVelocity;
            }
        }

        public static void SetAngularVelocity(BodyId bodyId, float angularVelocity)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);

            if (angularVelocity != 0.0f)
            {
                WakeBody(world, body);
            }

            ref var state = ref GetBodyState(world, body, out var success);
            if (success)
            {
                state.AngularVelocity = angularVelocity;
            }
        }

        public static void ApplyForce(BodyId bodyId, Vec2 force, Vec2 point, bool wake)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);

            if (wake && body.SetIndex >= SolverSetType.FirstSleepingSet)
            {
                WakeBody(world, body);
            }

            if (body.SetIndex == SolverSetType.AwakeSet)
            {
                BodySim bodySim = GetBodySim(world, body);
                bodySim.Force = B2Math.Add(bodySim.Force, force);
                bodySim.Torque += B2Math.Cross(B2Math.Sub(point, bodySim.Center), force);
            }
        }

        public static void ApplyForceToCenter(BodyId bodyId, Vec2 force, bool wake)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);

            if (wake && body.SetIndex >= SolverSetType.FirstSleepingSet)
            {
                WakeBody(world, body);
            }

            if (body.SetIndex == SolverSetType.AwakeSet)
            {
                BodySim bodySim = GetBodySim(world, body);
                bodySim.Force = B2Math.Add(bodySim.Force, force);
            }
        }

        public static void ApplyTorque(BodyId bodyId, float torque, bool wake)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);

            if (wake && body.SetIndex >= SolverSetType.FirstSleepingSet)
            {
                WakeBody(world, body);
            }

            if (body.SetIndex == SolverSetType.AwakeSet)
            {
                BodySim bodySim = GetBodySim(world, body);
                bodySim.Torque += torque;
            }
        }

        public static void ApplyLinearImpulse(BodyId bodyId, Vec2 impulse, Vec2 point, bool wake)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);

            if (wake && body.SetIndex >= SolverSetType.FirstSleepingSet)
            {
                WakeBody(world, body);
            }

            if (body.SetIndex == SolverSetType.AwakeSet)
            {
                int localIndex = body.LocalIndex;
                SolverSet set = world.SolverSetArray[SolverSetType.AwakeSet];
                Debug.Assert(0 <= localIndex && localIndex < set.States.Count);
                ref BodyState state = ref set.States[localIndex];
                BodySim bodySim = set.Sims[localIndex];
                state.LinearVelocity = B2Math.MulAdd(state.LinearVelocity, bodySim.InvMass, impulse);
                state.AngularVelocity += bodySim.InvInertia * B2Math.Cross(B2Math.Sub(point, bodySim.Center), impulse);
            }
        }

        public static void ApplyLinearImpulseToCenter(BodyId bodyId, Vec2 impulse, bool wake)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);

            if (wake && body.SetIndex >= SolverSetType.FirstSleepingSet)
            {
                WakeBody(world, body);
            }

            if (body.SetIndex == SolverSetType.AwakeSet)
            {
                int localIndex = body.LocalIndex;
                SolverSet set = world.SolverSetArray[SolverSetType.AwakeSet];
                Debug.Assert(0 <= localIndex && localIndex < set.States.Count);
                ref BodyState state = ref set.States[localIndex];
                BodySim bodySim = set.Sims[localIndex];
                state.LinearVelocity = B2Math.MulAdd(state.LinearVelocity, bodySim.InvMass, impulse);
            }
        }

        public static void ApplyAngularImpulse(BodyId bodyId, float impulse, bool wake)
        {
            Debug.Assert(bodyId.IsValid());
            World world = World.GetWorld(bodyId.World0);

            int id = bodyId.Index1 - 1;
            world.BodyArray.CheckIndex(id);
            Body body = world.BodyArray[id];
            Debug.Assert(body.Revision == bodyId.Revision);

            if (wake && body.SetIndex >= SolverSetType.FirstSleepingSet)
            {
                // this will not invalidate body pointer
                WakeBody(world, body);
            }

            if (body.SetIndex == SolverSetType.AwakeSet)
            {
                int localIndex = body.LocalIndex;
                SolverSet set = world.SolverSetArray[SolverSetType.AwakeSet];
                Debug.Assert(0 <= localIndex && localIndex < set.States.Count);
                ref BodyState state = ref set.States[localIndex];
                BodySim sim = set.Sims[localIndex];
                state.AngularVelocity += sim.InvInertia * impulse;
            }
        }

        public static BodyType GetType(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            return body.Type;
        }

        // Changing the body type is quite complex mainly due to joints.
        // Considerations:
        // - body and joints must be moved to the correct set
        // - islands must be updated
        // - graph coloring must be correct
        // - any body connected to a joint may be disabled
        // - joints between static bodies must go into the static set
        public static void SetType(BodyId bodyId, BodyType type)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);

            BodyType originalType = body.Type;
            if (originalType == type)
            {
                return;
            }

            if (body.SetIndex == SolverSetType.DisabledSet)
            {
                // Disabled bodies don't change solver sets or islands when they change type.
                body.Type = type;

                // Body type affects the mass
                UpdateBodyMassData(world, body);
                return;
            }

            // Destroy all contacts but don't wake bodies.
            bool wakeBodies = false;
            DestroyBodyContacts(world, body, wakeBodies);

            // Wake this body because we assume below that it is awake or static.
            WakeBody(world, body);

            // Unlink all joints and wake attached bodies.
            {
                int jointKey = body.HeadJointKey;
                while (jointKey != Core.NullIndex)
                {
                    int jointId = jointKey >> 1;
                    int edgeIndex = jointKey & 1;

                    Joint joint = world.JointArray[jointId];
                    if (joint.IslandId != Core.NullIndex)
                    {
                        Island.UnlinkJoint(world, joint);
                    }

                    // A body going from static to dynamic or kinematic goes to the awake set
                    // and other attached bodies must be awake as well. For consistency, this is
                    // done for all cases.
                    Body bodyA = world.BodyArray[joint.Edges[0].BodyId];
                    Body bodyB = world.BodyArray[joint.Edges[1].BodyId];
                    WakeBody(world, bodyA);
                    WakeBody(world, bodyB);

                    jointKey = joint.Edges[edgeIndex].NextKey;
                }
            }

            body.Type = type;

            if (originalType == BodyType.StaticBody)
            {
                // Body is going from static to dynamic or kinematic. It only makes sense to move it to the awake set.
                Debug.Assert(body.SetIndex == SolverSetType.StaticSet);

                SolverSet staticSet = world.SolverSetArray[SolverSetType.StaticSet];
                SolverSet awakeSet = world.SolverSetArray[SolverSetType.AwakeSet];

                // Transfer body to awake set
                SolverSet.TransferBody(world, awakeSet, staticSet, body);

                // Create island for body
                CreateIslandForBody(world, SolverSetType.AwakeSet, body);

                // Transfer static joints to awake set
                int jointKey = body.HeadJointKey;
                while (jointKey != Core.NullIndex)
                {
                    int jointId = jointKey >> 1;
                    int edgeIndex = jointKey & 1;

                    Joint joint = world.JointArray[jointId];

                    // Transfer the joint if it is in the static set
                    if (joint.SetIndex == SolverSetType.StaticSet)
                    {
                        SolverSet.TransferJoint(world, awakeSet, staticSet, joint);
                    }
                    else if (joint.SetIndex == SolverSetType.AwakeSet)
                    {
                        // In this case the joint must be re-inserted into the constraint graph to ensure the correct
                        // graph color.

                        // First transfer to the static set.
                        SolverSet.TransferJoint(world, staticSet, awakeSet, joint);

                        // Now transfer it back to the awake set and into the graph coloring.
                        SolverSet.TransferJoint(world, awakeSet, staticSet, joint);
                    }
                    else
                    {
                        // Otherwise the joint must be disabled.
                        Debug.Assert(joint.SetIndex == SolverSetType.DisabledSet);
                    }

                    jointKey = joint.Edges[edgeIndex].NextKey;
                }

                // Recreate shape proxies in movable tree.
                Transform transform = GetBodyTransformQuick(world, body);
                int shapeId = body.HeadShapeId;
                while (shapeId != Core.NullIndex)
                {
                    Shape shape = world.ShapeArray[shapeId];
                    shapeId = shape.NextShapeId;
                    Shape.DestroyShapeProxy(shape, world.BroadPhase);
                    bool forcePairCreation = true;
                    BodyType proxyType = type;
                    Shape.CreateShapeProxy(shape, world.BroadPhase, proxyType, transform, forcePairCreation);
                }
            }
            else if (type == BodyType.StaticBody)
            {
                // The body is going from dynamic/kinematic to static. It should be awake.
                Debug.Assert(body.SetIndex == SolverSetType.AwakeSet);

                SolverSet staticSet = world.SolverSetArray[SolverSetType.StaticSet];
                SolverSet awakeSet = world.SolverSetArray[SolverSetType.AwakeSet];

                // Transfer body to static set
                SolverSet.TransferBody(world, staticSet, awakeSet, body);

                // Remove body from island.
                RemoveBodyFromIsland(world, body);

                // Maybe transfer joints to static set.
                int jointKey = body.HeadJointKey;
                while (jointKey != Core.NullIndex)
                {
                    int jointId = jointKey >> 1;
                    int edgeIndex = jointKey & 1;

                    Joint joint = world.JointArray[jointId];
                    jointKey = joint.Edges[edgeIndex].NextKey;

                    int otherEdgeIndex = edgeIndex ^ 1;
                    Body otherBody = world.BodyArray[joint.Edges[otherEdgeIndex].BodyId];

                    // Skip disabled joint
                    if (joint.SetIndex == SolverSetType.DisabledSet)
                    {
                        // Joint is disable, should be connected to a disabled body
                        Debug.Assert(otherBody.SetIndex == SolverSetType.DisabledSet);
                        continue;
                    }

                    // Since the body was not static, the joint must be awake.
                    Debug.Assert(joint.SetIndex == SolverSetType.AwakeSet);

                    // Only transfer joint to static set if both bodies are static.
                    if (otherBody.SetIndex == SolverSetType.StaticSet)
                    {
                        SolverSet.TransferJoint(world, staticSet, awakeSet, joint);
                    }
                    else
                    {
                        // The other body must be awake.
                        Debug.Assert(otherBody.SetIndex == SolverSetType.AwakeSet);

                        // The joint must live in a graph color.
                        Debug.Assert(0 <= joint.ColorIndex && joint.ColorIndex < Core.GraphColorCount);

                        // In this case the joint must be re-inserted into the constraint graph to ensure the correct
                        // graph color.

                        // First transfer to the static set.
                        SolverSet.TransferJoint(world, staticSet, awakeSet, joint);

                        // Now transfer it back to the awake set and into the graph coloring.
                        SolverSet.TransferJoint(world, awakeSet, staticSet, joint);
                    }
                }

                // Recreate shape proxies in static tree.
                Transform transform = GetBodyTransformQuick(world, body);
                int shapeId = body.HeadShapeId;
                while (shapeId != Core.NullIndex)
                {
                    Shape shape = world.ShapeArray[shapeId];
                    shapeId = shape.NextShapeId;
                    Shape.DestroyShapeProxy(shape, world.BroadPhase);
                    bool forcePairCreation = true;
                    Shape.CreateShapeProxy(shape, world.BroadPhase, BodyType.StaticBody, transform, forcePairCreation);
                }
            }
            else
            {
                Debug.Assert(originalType == BodyType.DynamicBody || originalType == BodyType.KinematicBody);
                Debug.Assert(type == BodyType.DynamicBody || type == BodyType.KinematicBody);

                // Recreate shape proxies in static tree.
                Transform transform = GetBodyTransformQuick(world, body);
                int shapeId = body.HeadShapeId;
                while (shapeId != Core.NullIndex)
                {
                    Shape shape = world.ShapeArray[shapeId];
                    shapeId = shape.NextShapeId;
                    Shape.DestroyShapeProxy(shape, world.BroadPhase);
                    BodyType proxyType = type;
                    bool forcePairCreation = true;
                    Shape.CreateShapeProxy(shape, world.BroadPhase, proxyType, transform, forcePairCreation);
                }
            }

            // Relink all joints
            {
                int jointKey = body.HeadJointKey;
                while (jointKey != Core.NullIndex)
                {
                    int jointId = jointKey >> 1;
                    int edgeIndex = jointKey & 1;

                    Joint joint = world.JointArray[jointId];
                    jointKey = joint.Edges[edgeIndex].NextKey;

                    int otherEdgeIndex = edgeIndex ^ 1;
                    int otherBodyId = joint.Edges[otherEdgeIndex].BodyId;
                    world.BodyArray.CheckIndex(otherBodyId);
                    Body otherBody = world.BodyArray[otherBodyId];

                    if (otherBody.SetIndex == SolverSetType.DisabledSet)
                    {
                        continue;
                    }

                    if (body.Type == BodyType.StaticBody && otherBody.Type == BodyType.StaticBody)
                    {
                        continue;
                    }

                    Island.LinkJoint(world, joint, false);
                }

                Island.MergeAwakeIslands(world);
            }

            // Body type affects the mass
            UpdateBodyMassData(world, body);

            SolverSet.ValidateSolverSets(world);
        }

        public static void SetUserData(BodyId bodyId, object userData)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            body.UserData = userData;
        }

        public static object? GetUserData(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            return body.UserData;
        }

        public static float GetMass(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            BodySim bodySim = GetBodySim(world, body);
            return bodySim.Mass;
        }

        public static float GetRotationalInertia(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            BodySim bodySim = GetBodySim(world, body);
            return bodySim.Inertia;
        }

        public static Vec2 GetLocalCenterOfMass(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            BodySim bodySim = GetBodySim(world, body);
            return bodySim.LocalCenter;
        }

        public static Vec2 GetWorldCenterOfMass(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            BodySim bodySim = GetBodySim(world, body);
            return bodySim.Center;
        }

        public static void SetMassData(BodyId bodyId, MassData massData)
        {
            Debug.Assert(B2Math.IsValid(massData.Mass) && massData.Mass >= 0.0f);
            Debug.Assert(B2Math.IsValid(massData.RotationalInertia) && massData.RotationalInertia >= 0.0f);
            Debug.Assert(B2Math.Vec2_IsValid(massData.Center));

            var world = World.GetWorldLocked(bodyId.World0);

            Body body = GetBodyFullId(world, bodyId);
            BodySim bodySim = GetBodySim(world, body);

            bodySim.Mass = massData.Mass;
            bodySim.Inertia = massData.RotationalInertia;
            bodySim.LocalCenter = massData.Center;

            Vec2 center = B2Math.TransformPoint(bodySim.Transform, massData.Center);
            bodySim.Center = center;
            bodySim.Center0 = center;

            bodySim.InvMass = bodySim.Mass > 0.0f ? 1.0f / bodySim.Mass : 0.0f;
            bodySim.InvInertia = bodySim.Inertia > 0.0f ? 1.0f / bodySim.Inertia : 0.0f;
        }

        public static MassData GetMassData(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            BodySim bodySim = GetBodySim(world, body);
            MassData massData = new(bodySim.Mass, bodySim.LocalCenter, bodySim.Inertia);
            return massData;
        }

        public static void ApplyMassFromShapes(BodyId bodyId)
        {
            var world = World.GetWorldLocked(bodyId.World0);

            Body body = GetBodyFullId(world, bodyId);
            UpdateBodyMassData(world, body);
        }

        public static void SetAutomaticMass(BodyId bodyId, bool automaticMass)
        {
            var world = World.GetWorldLocked(bodyId.World0);

            Body body = GetBodyFullId(world, bodyId);
            body.AutomaticMass = automaticMass;
        }

        public static bool GetAutomaticMass(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            return body.AutomaticMass;
        }

        public static void SetLinearDamping(BodyId bodyId, float linearDamping)
        {
            Debug.Assert(B2Math.IsValid(linearDamping) && linearDamping >= 0.0f);

            var world = World.GetWorldLocked(bodyId.World0);

            Body body = GetBodyFullId(world, bodyId);
            BodySim bodySim = GetBodySim(world, body);
            bodySim.LinearDamping = linearDamping;
        }

        public static float GetLinearDamping(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            BodySim bodySim = GetBodySim(world, body);
            return bodySim.LinearDamping;
        }

        public static void SetAngularDamping(BodyId bodyId, float angularDamping)
        {
            Debug.Assert(B2Math.IsValid(angularDamping) && angularDamping >= 0.0f);

            var world = World.GetWorldLocked(bodyId.World0);

            Body body = GetBodyFullId(world, bodyId);
            BodySim bodySim = GetBodySim(world, body);
            bodySim.AngularDamping = angularDamping;
        }

        public static float GetAngularDamping(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            BodySim bodySim = GetBodySim(world, body);
            return bodySim.AngularDamping;
        }

        public static void SetGravityScale(BodyId bodyId, float gravityScale)
        {
            Debug.Assert(bodyId.IsValid());
            Debug.Assert(B2Math.IsValid(gravityScale));

            var world = World.GetWorldLocked(bodyId.World0);

            Body body = GetBodyFullId(world, bodyId);
            BodySim bodySim = GetBodySim(world, body);
            bodySim.GravityScale = gravityScale;
        }

        public static float GetGravityScale(BodyId bodyId)
        {
            Debug.Assert(bodyId.IsValid());
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            BodySim bodySim = GetBodySim(world, body);
            return bodySim.GravityScale;
        }

        public static bool IsAwake(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            return body.SetIndex == SolverSetType.AwakeSet;
        }

        public static void SetAwake(BodyId bodyId, bool awake)
        {
            var world = World.GetWorldLocked(bodyId.World0);

            Body body = GetBodyFullId(world, bodyId);

            if (awake && body.SetIndex >= SolverSetType.FirstSleepingSet)
            {
                WakeBody(world, body);
            }
            else if (awake == false && body.SetIndex == SolverSetType.AwakeSet)
            {
                world.IslandArray.CheckIndex(body.IslandId);
                Island island = world.IslandArray[body.IslandId];
                if (island.ConstraintRemoveCount > 0)
                {
                    Island.SplitIsland(world, body.IslandId);
                }

                SolverSet.TrySleepIsland(world, body.IslandId);
            }
        }

        public static bool IsEnabled(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            return body.SetIndex != SolverSetType.DisabledSet;
        }

        public static bool IsSleepEnabled(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            return body.EnableSleep;
        }

        public static void SetSleepThreshold(BodyId bodyId, float sleepThreshold)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            body.SleepThreshold = sleepThreshold;
        }

        public static float GetSleepThreshold(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            return body.SleepThreshold;
        }

        public static void SetEnableSleep(BodyId bodyId, bool enableSleep)
        {
            var world = World.GetWorldLocked(bodyId.World0);

            Body body = GetBodyFullId(world, bodyId);
            body.EnableSleep = enableSleep;

            if (enableSleep == false)
            {
                WakeBody(world, body);
            }
        }

        // Disabling a body requires a lot of detailed bookkeeping, but it is a valuable feature.
        // The most challenging aspect that joints may connect to bodies that are not disabled.
        public static void Disable(BodyId bodyId)
        {
            var world = World.GetWorldLocked(bodyId.World0);

            Body body = GetBodyFullId(world, bodyId);
            if (body.SetIndex == SolverSetType.DisabledSet)
            {
                return;
            }

            // Destroy contacts and wake bodies touching this body. This avoid floating bodies.
            // This is necessary even for static bodies.
            bool wakeBodies = true;
            DestroyBodyContacts(world, body, wakeBodies);

            // Disabled bodies are not in an island.
            RemoveBodyFromIsland(world, body);

            // Remove shapes from broad-phase
            int shapeId = body.HeadShapeId;
            while (shapeId != Core.NullIndex)
            {
                Shape shape = world.ShapeArray[shapeId];
                shapeId = shape.NextShapeId;
                Shape.DestroyShapeProxy(shape, world.BroadPhase);
            }

            // Transfer simulation data to disabled set
            world.SolverSetArray.CheckIndex(body.SetIndex);
            SolverSet set = world.SolverSetArray[body.SetIndex];
            SolverSet disabledSet = world.SolverSetArray[SolverSetType.DisabledSet];

            // Transfer body sim
            SolverSet.TransferBody(world, disabledSet, set, body);

            // Unlink joints and transfer
            int jointKey = body.HeadJointKey;
            while (jointKey != Core.NullIndex)
            {
                int jointId = jointKey >> 1;
                int edgeIndex = jointKey & 1;

                Joint joint = world.JointArray[jointId];
                jointKey = joint.Edges[edgeIndex].NextKey;

                // joint may already be disabled by other body
                if (joint.SetIndex == SolverSetType.DisabledSet)
                {
                    continue;
                }

                Debug.Assert(joint.SetIndex == set.SetIndex || set.SetIndex == SolverSetType.StaticSet);

                // Remove joint from island
                if (joint.IslandId != Core.NullIndex)
                {
                    Island.UnlinkJoint(world, joint);
                }

                // Transfer joint to disabled set
                world.SolverSetArray.CheckIndex(joint.SetIndex);
                SolverSet jointSet = world.SolverSetArray[joint.SetIndex];
                SolverSet.TransferJoint(world, disabledSet, jointSet, joint);
            }

            SolverSet.ValidateConnectivity(world);
            SolverSet.ValidateSolverSets(world);
        }

        public static void Enable(BodyId bodyId)
        {
            var world = World.GetWorldLocked(bodyId.World0);

            Body body = GetBodyFullId(world, bodyId);
            if (body.SetIndex != SolverSetType.DisabledSet)
            {
                return;
            }

            SolverSet disabledSet = world.SolverSetArray[SolverSetType.DisabledSet];
            int setId = body.Type == BodyType.StaticBody ? SolverSetType.StaticSet : SolverSetType.AwakeSet;
            SolverSet targetSet = world.SolverSetArray[setId];

            SolverSet.TransferBody(world, targetSet, disabledSet, body);

            Transform transform = GetBodyTransformQuick(world, body);

            // Add shapes to broad-phase
            BodyType proxyType = body.Type;
            bool forcePairCreation = true;
            int shapeId = body.HeadShapeId;
            while (shapeId != Core.NullIndex)
            {
                Shape shape = world.ShapeArray[shapeId];
                shapeId = shape.NextShapeId;

                Shape.CreateShapeProxy(shape, world.BroadPhase, proxyType, transform, forcePairCreation);
            }

            if (setId != SolverSetType.StaticSet)
            {
                CreateIslandForBody(world, setId, body);
            }

            // Transfer joints. If the other body is disabled, don't transfer.
            // If the other body is sleeping, wake it.
            bool mergeIslands = false;
            int jointKey = body.HeadJointKey;
            while (jointKey != Core.NullIndex)
            {
                int jointId = jointKey >> 1;
                int edgeIndex = jointKey & 1;

                Joint joint = world.JointArray[jointId];
                Debug.Assert(joint.SetIndex == SolverSetType.DisabledSet);
                Debug.Assert(joint.IslandId == Core.NullIndex);

                jointKey = joint.Edges[edgeIndex].NextKey;

                Body bodyA = world.BodyArray[joint.Edges[0].BodyId];
                Body bodyB = world.BodyArray[joint.Edges[1].BodyId];

                if (bodyA.SetIndex == SolverSetType.DisabledSet || bodyB.SetIndex == SolverSetType.DisabledSet)
                {
                    // one body is still disabled
                    continue;
                }

                // Transfer joint first
                int jointSetId;
                if (bodyA.SetIndex == SolverSetType.StaticSet && bodyB.SetIndex == SolverSetType.StaticSet)
                {
                    jointSetId = SolverSetType.StaticSet;
                }
                else if (bodyA.SetIndex == SolverSetType.StaticSet)
                {
                    jointSetId = bodyB.SetIndex;
                }
                else
                {
                    jointSetId = bodyA.SetIndex;
                }

                world.SolverSetArray.CheckIndex(jointSetId);
                SolverSet jointSet = world.SolverSetArray[jointSetId];
                SolverSet.TransferJoint(world, jointSet, disabledSet, joint);

                // Now that the joint is in the correct set, I can link the joint in the island.
                if (jointSetId != SolverSetType.StaticSet)
                {
                    Island.LinkJoint(world, joint, mergeIslands);
                }
            }

            // Now merge islands
            Island.MergeAwakeIslands(world);

            SolverSet.ValidateSolverSets(world);
        }

        public static void SetFixedRotation(BodyId bodyId, bool flag)
        {
            var world = World.GetWorldLocked(bodyId.World0);

            Body body = GetBodyFullId(world, bodyId);
            if (body.FixedRotation != flag)
            {
                body.FixedRotation = flag;

                ref var state = ref GetBodyState(world, body, out var success);
                if (success)
                {
                    state.AngularVelocity = 0.0f;
                }

                UpdateBodyMassData(world, body);
            }
        }

        public static bool IsFixedRotation(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            return body.FixedRotation;
        }

        public static void SetBullet(BodyId bodyId, bool flag)
        {
            var world = World.GetWorldLocked(bodyId.World0);

            Body body = GetBodyFullId(world, bodyId);
            BodySim bodySim = GetBodySim(world, body);
            bodySim.IsBullet = flag;
        }

        public static bool IsBullet(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            BodySim bodySim = GetBodySim(world, body);
            return bodySim.IsBullet;
        }

        public static void EnableHitEvents(BodyId bodyId, bool enableHitEvents)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            int shapeId = body.HeadShapeId;
            while (shapeId != Core.NullIndex)
            {
                Shape shape = world.ShapeArray[shapeId];
                shape.EnableHitEvents = enableHitEvents;
                shapeId = shape.NextShapeId;
            }
        }

        public static int GetShapeCount(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            return body.ShapeCount;
        }

        public static ShapeId GetShape(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            int shapeId = body.HeadShapeId;

            if (shapeId != Core.NullIndex)
            {
                Shape shape = world.ShapeArray[shapeId];
                return (shape.Id + 1, bodyId.World0, shape.Revision);
            }

            return ShapeId.NullId;
        }

        public static int GetShapes(BodyId bodyId, Span<ShapeId> shapeArray, int capacity)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            int shapeId = body.HeadShapeId;
            int shapeCount = 0;
            while (shapeId != Core.NullIndex && shapeCount < capacity)
            {
                Shape shape = world.ShapeArray[shapeId];
                ShapeId id = (shape.Id + 1, bodyId.World0, shape.Revision);
                shapeArray[shapeCount] = id;
                shapeCount += 1;

                shapeId = shape.NextShapeId;
            }

            return shapeCount;
        }

        public static int GetJointCount(BodyId bodyId)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            return body.JointCount;
        }

        public static int GetJoints(BodyId bodyId, Span<JointId> jointArray, int capacity)
        {
            World world = World.GetWorld(bodyId.World0);
            Body body = GetBodyFullId(world, bodyId);
            int jointKey = body.HeadJointKey;

            int jointCount = 0;
            while (jointKey != Core.NullIndex && jointCount < capacity)
            {
                int jointId = jointKey >> 1;
                int edgeIndex = jointKey & 1;

                Joint joint = Joint.GetJoint(world, jointId);

                JointId id = new(jointId + 1, bodyId.World0, (ushort)joint.Revision);
                jointArray[jointCount] = id;
                jointCount += 1;

                jointKey = joint.Edges[edgeIndex].NextKey;
            }

            return jointCount;
        }

        public static bool ShouldBodiesCollide(World world, Body bodyA, Body bodyB)
        {
            if (bodyA.Type != BodyType.DynamicBody && bodyB.Type != BodyType.DynamicBody)
            {
                return false;
            }

            int jointKey;
            int otherBodyId;
            if (bodyA.JointCount < bodyB.JointCount)
            {
                jointKey = bodyA.HeadJointKey;
                otherBodyId = bodyB.Id;
            }
            else
            {
                jointKey = bodyB.HeadJointKey;
                otherBodyId = bodyA.Id;
            }

            while (jointKey != Core.NullIndex)
            {
                int jointId = jointKey >> 1;
                int edgeIndex = jointKey & 1;
                int otherEdgeIndex = edgeIndex ^ 1;

                Joint joint = Joint.GetJoint(world, jointId);
                if (joint.CollideConnected == false && joint.Edges[otherEdgeIndex].BodyId == otherBodyId)
                {
                    return false;
                }

                jointKey = joint.Edges[edgeIndex].NextKey;
            }

            return true;
        }
    }

    // The body state is designed for fast conversion to and from SIMD via scatter-gather.
    // Only awake dynamic and kinematic bodies have a body state.
    // This is used in the performance critical constraint solver
    //
    // 32 bytes
}