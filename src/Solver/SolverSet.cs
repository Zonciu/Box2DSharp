using System.Diagnostics;

namespace Box2DSharp
{
    /// <summary>
    /// This holds solver set data. The following sets are used:
    /// <br/>- static set for all static bodies (no contacts or joints)
    /// <br/>- active set for all active bodies with body states (no contacts or joints)
    /// <br/>- disabled set for disabled bodies and their joints
    /// <br/>- all further sets are sleeping island sets along with their contacts and joints
    /// <br/>The purpose of solver sets is to achieve high memory locality.
    /// https://www.youtube.com/watch?v=nZNd5FjSquk
    /// </summary>
    public class SolverSet
    {
        /// <summary>
        /// Body array. Empty for unused set.
        /// </summary>
        public BodySimArray Sims = new();

        /// <summary>
        /// Body state only exists for active set
        /// </summary>
        public BodyStateArray States = new();

        /// <summary>
        /// This holds sleeping/disabled joints. Empty for static/active set.
        /// </summary>
        public JointArray Joints = new();

        /// <summary>
        /// This holds all contacts for sleeping sets.
        /// This holds non-touching contacts for the awake set.
        /// </summary>
        public ContactArray Contacts = new();

        // The awake set has an array of islands. Sleeping sets normally have a single islands. However, joints
        // created between sleeping sets causes the sets to merge, leaving them with multiple islands. These sleeping
        // islands will be naturally merged with the set is woken.
        // The static and disabled sets have no islands.
        // Islands live in the solver sets to limit the number of islands that need to be considered for sleeping.
        public IslandArray Islands = new();

        // Aligns with b2World::solverSetIdPool. Used to create a stable id for body/contact/joint/islands.
        public int SetIndex;

        #region Validate

        // When validating islands ids I have to compare the root island
        // ids because islands are not merged until the next time step.
        public static int GetRootIslandId(World world, int islandId)
        {
            if (islandId == Core.NullIndex)
            {
                return Core.NullIndex;
            }

            var islands = world.IslandArray;
            islands.CheckIndex(islandId);
            Island island = islands[islandId];

            int rootId = islandId;
            Island rootIsland = island;
            while (rootIsland.ParentIsland != Core.NullIndex)
            {
                islands.CheckIndex(rootIsland.ParentIsland);
                Island parent = islands[rootIsland.ParentIsland];
                rootId = rootIsland.ParentIsland;
                rootIsland = parent;
            }

            return rootId;
        }

        // This validates island graph connectivity for each body
        [Conditional("DEBUG")]
        public static void ValidateConnectivity(World world)
        {
            var bodies = world.BodyArray;
            int bodyCapacity = bodies.Count;

            for (int bodyIndex = 0; bodyIndex < bodyCapacity; ++bodyIndex)
            {
                Body body = bodies[bodyIndex];
                if (body.Id == Core.NullIndex)
                {
                    world.BodyIdPool.ValidateFreeId(bodyIndex);
                    continue;
                }

                Debug.Assert(bodyIndex == body.Id);

                // Need to get the root island because islands are not merged until the next time step
                int bodyIslandId = GetRootIslandId(world, body.IslandId);
                int bodySetIndex = body.SetIndex;

                int contactKey = body.HeadContactKey;
                while (contactKey != Core.NullIndex)
                {
                    int contactId = contactKey >> 1;
                    int edgeIndex = contactKey & 1;

                    world.ContactArray.CheckIndex(contactId);
                    Contact contact = world.ContactArray[contactId];

                    bool touching = (contact.Flags & ContactFlags.ContactTouchingFlag) != 0;
                    if (touching && (contact.Flags & ContactFlags.ContactSensorFlag) == 0)
                    {
                        if (bodySetIndex != SolverSetType.StaticSet)
                        {
                            int contactIslandId = GetRootIslandId(world, contact.IslandId);
                            Debug.Assert(contactIslandId == bodyIslandId);
                        }
                    }
                    else
                    {
                        Debug.Assert(contact.IslandId == Core.NullIndex);
                    }

                    contactKey = contact.Edges[edgeIndex].NextKey;
                }

                int jointKey = body.HeadJointKey;
                while (jointKey != Core.NullIndex)
                {
                    int jointId = jointKey >> 1;
                    int edgeIndex = jointKey & 1;

                    world.JointArray.CheckIndex(jointId);
                    Joint joint = world.JointArray[jointId];

                    int otherEdgeIndex = edgeIndex ^ 1;

                    world.BodyArray.CheckIndex(joint.Edges[otherEdgeIndex].BodyId);
                    Body otherBody = world.BodyArray[joint.Edges[otherEdgeIndex].BodyId];

                    if (bodySetIndex == SolverSetType.DisabledSet || otherBody.SetIndex == SolverSetType.DisabledSet)
                    {
                        Debug.Assert(joint.IslandId == Core.NullIndex);
                    }
                    else if (bodySetIndex == SolverSetType.StaticSet)
                    {
                        if (otherBody.SetIndex == SolverSetType.StaticSet)
                        {
                            Debug.Assert(joint.IslandId == Core.NullIndex);
                        }
                    }
                    else
                    {
                        int jointIslandId = GetRootIslandId(world, joint.IslandId);
                        Debug.Assert(jointIslandId == bodyIslandId);
                    }

                    jointKey = joint.Edges[edgeIndex].NextKey;
                }
            }
        }

        // Validates solver sets, but not island connectivity
        [Conditional("DEBUG")]
        public static void ValidateSolverSets(World world)
        {
            Debug.Assert(world.BodyIdPool.GetIdCapacity() == world.BodyArray.Count);
            Debug.Assert(world.ContactIdPool.GetIdCapacity() == world.ContactArray.Count);
            Debug.Assert(world.JointIdPool.GetIdCapacity() == world.JointArray.Count);
            Debug.Assert(world.IslandIdPool.GetIdCapacity() == world.IslandArray.Count);
            Debug.Assert(world.SolverSetIdPool.GetIdCapacity() == world.SolverSetArray.Count);

            int activeSetCount = 0;
            int totalBodyCount = 0;
            int totalJointCount = 0;
            int totalContactCount = 0;
            int totalIslandCount = 0;

            // Validate all solver sets
            int setCount = world.SolverSetArray.Count;
            for (int setIndex = 0; setIndex < setCount; ++setIndex)
            {
                SolverSet set = world.SolverSetArray[setIndex];
                if (set.SetIndex != Core.NullIndex)
                {
                    activeSetCount += 1;

                    if (setIndex == SolverSetType.StaticSet)
                    {
                        Debug.Assert(set.Contacts.Count == 0);
                        Debug.Assert(set.Islands.Count == 0);
                        Debug.Assert(set.States.Count == 0);
                    }
                    else if (setIndex == SolverSetType.AwakeSet)
                    {
                        Debug.Assert(set.Sims.Count == set.States.Count);
                        Debug.Assert(set.Joints.Count == 0);
                    }
                    else if (setIndex == SolverSetType.DisabledSet)
                    {
                        Debug.Assert(set.Islands.Count == 0);
                        Debug.Assert(set.States.Count == 0);
                    }
                    else
                    {
                        Debug.Assert(set.States.Count == 0);
                    }

                    // Validate bodies
                    {
                        var bodies = world.BodyArray;
                        Debug.Assert(set.Sims.Count >= 0);
                        totalBodyCount += set.Sims.Count;
                        for (int i = 0; i < set.Sims.Count; ++i)
                        {
                            BodySim bodySim = set.Sims.Data[i];

                            int bodyId = bodySim.BodyId;
                            bodies.CheckIndex(bodyId);
                            Body body = bodies[bodyId];
                            Debug.Assert(body.SetIndex == setIndex);
                            Debug.Assert(body.LocalIndex == i);

                            if (setIndex == SolverSetType.DisabledSet)
                            {
                                Debug.Assert(body.HeadContactKey == Core.NullIndex);
                            }

                            // Validate body shapes
                            int prevShapeId = Core.NullIndex;
                            int shapeId = body.HeadShapeId;
                            while (shapeId != Core.NullIndex)
                            {
                                world.ShapeArray.CheckId(shapeId);
                                Shape shape = world.ShapeArray[shapeId];
                                Debug.Assert(shape.PrevShapeId == prevShapeId);

                                if (setIndex == SolverSetType.DisabledSet)
                                {
                                    Debug.Assert(shape.ProxyKey == Core.NullIndex);
                                }
                                else if (setIndex == SolverSetType.StaticSet)
                                {
                                    Debug.Assert(BroadPhase.ProxyType(shape.ProxyKey) == (int)BodyType.StaticBody);
                                }
                                else
                                {
                                    BodyType proxyType = (BodyType)BroadPhase.ProxyType(shape.ProxyKey);
                                    Debug.Assert(proxyType == BodyType.KinematicBody || proxyType == BodyType.DynamicBody);
                                }

                                prevShapeId = shapeId;
                                shapeId = shape.NextShapeId;
                            }

                            // Validate body contacts
                            int contactKey = body.HeadContactKey;
                            while (contactKey != Core.NullIndex)
                            {
                                int contactId = contactKey >> 1;
                                int edgeIndex = contactKey & 1;

                                world.ContactArray.CheckIndex(contactId);
                                Contact contact = world.ContactArray[contactId];
                                Debug.Assert(contact.SetIndex != SolverSetType.StaticSet);
                                Debug.Assert(contact.Edges[0].BodyId == bodyId || contact.Edges[1].BodyId == bodyId);
                                contactKey = contact.Edges[edgeIndex].NextKey;
                            }

                            // Validate body joints
                            int jointKey = body.HeadJointKey;
                            while (jointKey != Core.NullIndex)
                            {
                                int jointId = jointKey >> 1;
                                int edgeIndex = jointKey & 1;

                                world.JointArray.CheckIndex(jointId);
                                Joint joint = world.JointArray[jointId];

                                int otherEdgeIndex = edgeIndex ^ 1;

                                world.BodyArray.CheckIndex(joint.Edges[otherEdgeIndex].BodyId);
                                Body otherBody = world.BodyArray[joint.Edges[otherEdgeIndex].BodyId];

                                if (setIndex == SolverSetType.DisabledSet || otherBody.SetIndex == SolverSetType.DisabledSet)
                                {
                                    Debug.Assert(joint.SetIndex == SolverSetType.DisabledSet);
                                }
                                else if (setIndex == SolverSetType.StaticSet && otherBody.SetIndex == SolverSetType.StaticSet)
                                {
                                    Debug.Assert(joint.SetIndex == SolverSetType.StaticSet);
                                }
                                else if (setIndex == SolverSetType.AwakeSet)
                                {
                                    Debug.Assert(joint.SetIndex == SolverSetType.AwakeSet);
                                }
                                else if (setIndex >= SolverSetType.FirstSleepingSet)
                                {
                                    Debug.Assert(joint.SetIndex == setIndex);
                                }

                                JointSim jointSim = Joint.GetJointSim(world, joint);
                                Debug.Assert(jointSim.JointId == jointId);
                                Debug.Assert(jointSim.BodyIdA == joint.Edges[0].BodyId);
                                Debug.Assert(jointSim.BodyIdB == joint.Edges[1].BodyId);

                                jointKey = joint.Edges[edgeIndex].NextKey;
                            }
                        }
                    }

                    // Validate contacts
                    {
                        var contacts = world.ContactArray;
                        Debug.Assert(set.Contacts.Count >= 0);
                        totalContactCount += set.Contacts.Count;
                        for (int i = 0; i < set.Contacts.Count; ++i)
                        {
                            ContactSim contactSim = set.Contacts.Data[i];
                            contacts.CheckIndex(contactSim.ContactId);
                            Contact contact = contacts[contactSim.ContactId];
                            if (setIndex == SolverSetType.AwakeSet)
                            {
                                // contact should be non-touching if awake
                                // or it could be this contact hasn't been transferred yet
                                Debug.Assert(contactSim.Manifold.PointCount == 0 || (contactSim.SimFlags & ContactSimFlags.StartedTouching) != 0);
                            }

                            Debug.Assert(contact.SetIndex == setIndex);
                            Debug.Assert(contact.ColorIndex == Core.NullIndex);
                            Debug.Assert(contact.LocalIndex == i);
                        }
                    }

                    // Validate joints
                    {
                        var joints = world.JointArray;
                        Debug.Assert(set.Joints.Count >= 0);
                        totalJointCount += set.Joints.Count;
                        for (int i = 0; i < set.Joints.Count; ++i)
                        {
                            JointSim jointSim = set.Joints.Data[i];
                            joints.CheckIndex(jointSim.JointId);
                            Joint joint = joints[jointSim.JointId];
                            Debug.Assert(joint.SetIndex == setIndex);
                            Debug.Assert(joint.ColorIndex == Core.NullIndex);
                            Debug.Assert(joint.LocalIndex == i);
                        }
                    }

                    // Validate islands
                    {
                        var islands = world.IslandArray;
                        Debug.Assert(set.Islands.Count >= 0);
                        totalIslandCount += set.Islands.Count;
                        for (int i = 0; i < set.Islands.Count; ++i)
                        {
                            IslandSim islandSim = set.Islands.Data[i];
                            islands.CheckIndex(islandSim.IslandId);
                            Island island = islands[islandSim.IslandId];
                            Debug.Assert(island.SetIndex == setIndex);
                            Debug.Assert(island.LocalIndex == i);
                        }
                    }
                }
                else
                {
                    Debug.Assert(set.Sims.Count == 0);
                    Debug.Assert(set.Contacts.Count == 0);
                    Debug.Assert(set.Joints.Count == 0);
                    Debug.Assert(set.Islands.Count == 0);
                    Debug.Assert(set.States.Count == 0);
                }
            }

            int setIdCount = (world.SolverSetIdPool.GetIdCount());
            Debug.Assert(activeSetCount == setIdCount);

            int bodyIdCount = (world.BodyIdPool.GetIdCount());
            Debug.Assert(totalBodyCount == bodyIdCount);

            int islandIdCount = (world.IslandIdPool.GetIdCount());
            Debug.Assert(totalIslandCount == islandIdCount);

            // Validate constraint graph
            for (int colorIndex = 0; colorIndex < Core.GraphColorCount; ++colorIndex)
            {
                GraphColor color = world.ConstraintGraph.Colors[colorIndex];
                {
                    var contacts = world.ContactArray;
                    Debug.Assert(color.Contacts.Count >= 0);
                    totalContactCount += color.Contacts.Count;
                    for (int i = 0; i < color.Contacts.Count; ++i)
                    {
                        ContactSim contactSim = color.Contacts[i];
                        contacts.CheckIndex(contactSim.ContactId);
                        var contact = contacts[contactSim.ContactId];

                        // contact should be touching in the constraint graph or awaiting transfer to non-touching
                        Debug.Assert(contactSim.Manifold.PointCount > 0 || (contactSim.SimFlags & (ContactSimFlags.StoppedTouching | ContactSimFlags.Disjoint)) != 0);
                        Debug.Assert(contact.SetIndex == SolverSetType.AwakeSet);
                        Debug.Assert(contact.ColorIndex == colorIndex);
                        Debug.Assert(contact.LocalIndex == i);

                        int bodyIdA = contact.Edges[0].BodyId;
                        int bodyIdB = contact.Edges[1].BodyId;
                        world.BodyArray.CheckIndex(bodyIdA);
                        world.BodyArray.CheckIndex(bodyIdB);

                        if (colorIndex < Core.OverflowIndex)
                        {
                            Body bodyA = world.BodyArray[bodyIdA];
                            Body bodyB = world.BodyArray[bodyIdB];
                            Debug.Assert(color.BodySet.GetBit(bodyIdA) == (bodyA.Type != BodyType.StaticBody));
                            Debug.Assert(color.BodySet.GetBit(bodyIdB) == (bodyB.Type != BodyType.StaticBody));
                        }
                    }
                }

                {
                    var joints = world.JointArray;
                    Debug.Assert(color.Joints.Count >= 0);
                    totalJointCount += color.Joints.Count;
                    for (int i = 0; i < color.Joints.Count; ++i)
                    {
                        JointSim jointSim = color.Joints.Data[i];
                        joints.CheckIndex(jointSim.JointId);
                        Joint joint = joints[jointSim.JointId];
                        Debug.Assert(joint.SetIndex == SolverSetType.AwakeSet);
                        Debug.Assert(joint.ColorIndex == colorIndex);
                        Debug.Assert(joint.LocalIndex == i);

                        int bodyIdA = joint.Edges[0].BodyId;
                        int bodyIdB = joint.Edges[1].BodyId;
                        world.BodyArray.CheckIndex(bodyIdA);
                        world.BodyArray.CheckIndex(bodyIdB);

                        if (colorIndex < Core.OverflowIndex)
                        {
                            Body bodyA = world.BodyArray[bodyIdA];
                            Body bodyB = world.BodyArray[bodyIdB];
                            Debug.Assert(color.BodySet.GetBit(bodyIdA) == (bodyA.Type != BodyType.StaticBody));
                            Debug.Assert(color.BodySet.GetBit(bodyIdB) == (bodyB.Type != BodyType.StaticBody));
                        }
                    }
                }
            }

            int contactIdCount = world.ContactIdPool.GetIdCount();
            Debug.Assert(totalContactCount == contactIdCount);
            Debug.Assert(totalContactCount == world.BroadPhase.PairSet.Count);

            int jointIdCount = world.JointIdPool.GetIdCount();
            Debug.Assert(totalJointCount == jointIdCount);
        }

        // Validate contact touching status.
        [Conditional("DEBUG")]
        public static void ValidateContacts(World world)
        {
            int contactCount = world.ContactArray.Count;
            Debug.Assert(contactCount == world.ContactIdPool.GetIdCapacity());
            int allocatedContactCount = 0;

            for (int contactIndex = 0; contactIndex < contactCount; ++contactIndex)
            {
                Contact contact = world.ContactArray[contactIndex];
                if (contact.ContactId == Core.NullIndex)
                {
                    continue;
                }

                Debug.Assert(contact.ContactId == contactIndex);

                allocatedContactCount += 1;

                bool touching = (contact.Flags & ContactFlags.ContactTouchingFlag) != 0;
                bool sensorTouching = (contact.Flags & ContactFlags.ContactSensorTouchingFlag) != 0;
                bool isSensor = (contact.Flags & ContactFlags.ContactSensorFlag) != 0;

                Debug.Assert(touching == false || sensorTouching == false);
                Debug.Assert(touching == false || isSensor == false);

                int setId = contact.SetIndex;

                if (setId == SolverSetType.AwakeSet)
                {
                    // If touching and not a sensor
                    if (touching && isSensor == false)
                    {
                        Debug.Assert(0 <= contact.ColorIndex && contact.ColorIndex < Core.GraphColorCount);
                    }
                    else
                    {
                        Debug.Assert(contact.ColorIndex == Core.NullIndex);
                    }
                }
                else if (setId >= SolverSetType.FirstSleepingSet)
                {
                    // Only touching contacts allowed in a sleeping set
                    Debug.Assert(touching == true && isSensor == false);
                }
                else
                {
                    // Sleeping and non-touching contacts or sensor contacts belong in the disabled set
                    Debug.Assert(touching == false && setId == SolverSetType.DisabledSet);
                }

                ContactSim contactSim = Contact.GetContactSim(world, contact);
                Debug.Assert(contactSim.ContactId == contactIndex);
                Debug.Assert(contactSim.BodyIdA == contact.Edges[0].BodyId);
                Debug.Assert(contactSim.BodyIdB == contact.Edges[1].BodyId);

                // Sim touching is true for solid and sensor contacts
                bool simTouching = (contactSim.SimFlags & ContactSimFlags.TouchingFlag) != 0;
                Debug.Assert(touching == simTouching || sensorTouching == simTouching);

                Debug.Assert(0 <= contactSim.Manifold.PointCount && contactSim.Manifold.PointCount <= 2);
            }

            int contactIdCount = (world.ContactIdPool.GetIdCount());
            Debug.Assert(allocatedContactCount == contactIdCount);
        }

        #endregion Validate

        public static void DestroySolverSet(World world, int setIndex)
        {
            ref SolverSet set = ref world.SolverSetArray[setIndex];
            set.Sims.Dispose();
            set.States.Dispose();
            set.Contacts.Dispose();
            set.Joints.Dispose();
            set.Islands.Dispose();
            world.SolverSetIdPool.FreeId(setIndex);
            set = new SolverSet
            {
                SetIndex = Core.NullIndex
            };
        }

        // Wake a solver set. Does not merge islands.
        // Contacts can be in several places:
        // 1. non-touching contacts in the disabled set
        // 2. non-touching contacts already in the awake set
        // 3. touching contacts in the sleeping set
        // This handles contact types 1 and 3. Type 2 doesn't need any action.
        public static void WakeSolverSet(World world, int setIndex)
        {
            Debug.Assert(setIndex >= SolverSetType.FirstSleepingSet);
            world.SolverSetArray.CheckIndex(setIndex);
            SolverSet set = world.SolverSetArray[setIndex];
            SolverSet awakeSet = world.SolverSetArray[SolverSetType.AwakeSet];
            SolverSet disabledSet = world.SolverSetArray[SolverSetType.DisabledSet];

            var bodies = world.BodyArray;
            var contacts = world.ContactArray;

            int bodyCount = set.Sims.Count;
            for (int i = 0; i < bodyCount; ++i)
            {
                BodySim simSrc = set.Sims.Data[i];

                Body body = bodies[simSrc.BodyId];
                Debug.Assert(body.SetIndex == setIndex);
                body.SetIndex = SolverSetType.AwakeSet;
                body.LocalIndex = awakeSet.Sims.Count;

                // Reset sleep timer
                body.SleepTime = 0.0f;

                BodySim simDst = awakeSet.Sims.AddBodySim();
                simSrc.CopyTo(simDst);

                ref BodyState state = ref awakeSet.States.AddBodyState();
                state = BodyState.Identity;

                // move non-touching contacts from disabled set to awake set
                int contactKey = body.HeadContactKey;
                while (contactKey != Core.NullIndex)
                {
                    int edgeIndex = contactKey & 1;
                    int contactId = contactKey >> 1;

                    contacts.CheckIndex(contactId);
                    Contact contact = contacts[contactId];

                    contactKey = contact.Edges[edgeIndex].NextKey;

                    if (contact.SetIndex != SolverSetType.DisabledSet)
                    {
                        Debug.Assert(contact.SetIndex == SolverSetType.AwakeSet || contact.SetIndex == setIndex);
                        continue;
                    }

                    int localIndex = contact.LocalIndex;
                    Debug.Assert(0 <= localIndex && localIndex < disabledSet.Contacts.Count);
                    ContactSim contactSim = disabledSet.Contacts.Data[localIndex];

                    Debug.Assert((contact.Flags & ContactFlags.ContactTouchingFlag) == 0 && contactSim.Manifold.PointCount == 0);

                    contact.SetIndex = SolverSetType.AwakeSet;
                    contact.LocalIndex = awakeSet.Contacts.Count;
                    ContactSim awakeContactSim = awakeSet.Contacts.AddContact();
                    contactSim.CopyTo(awakeContactSim);

                    int movedLocalIndex = disabledSet.Contacts.RemoveContact(localIndex);
                    if (movedLocalIndex != Core.NullIndex)
                    {
                        // fix moved element
                        ContactSim movedContact = disabledSet.Contacts.Data[localIndex];
                        int movedId = movedContact.ContactId;
                        contacts.CheckIndex(movedId);
                        Debug.Assert(contacts[movedId].LocalIndex == movedLocalIndex);
                        contacts[movedId].LocalIndex = localIndex;
                    }
                }
            }

            // transfer touching contacts from sleeping set to contact graph
            {
                int contactCount = set.Contacts.Count;
                for (int i = 0; i < contactCount; ++i)
                {
                    ContactSim contactSim = set.Contacts.Data[i];
                    Contact contact = contacts[contactSim.ContactId];
                    Debug.Assert((contact.Flags & ContactFlags.ContactTouchingFlag) != 0);
                    Debug.Assert((contactSim.SimFlags & ContactSimFlags.TouchingFlag) != 0);
                    Debug.Assert(contactSim.Manifold.PointCount > 0);
                    Debug.Assert(contact.SetIndex == setIndex);
                    ConstraintGraph.AddContactToGraph(world, contactSim, contact);
                    contact.SetIndex = SolverSetType.AwakeSet;
                }
            }

            // transfer joints from sleeping set to awake set
            {
                var joints = world.JointArray;
                int jointCount = set.Joints.Count;
                for (int i = 0; i < jointCount; ++i)
                {
                    JointSim jointSim = set.Joints.Data[i];
                    Joint joint = joints[jointSim.JointId];
                    Debug.Assert(joint.SetIndex == setIndex);
                    ConstraintGraph.AddJointToGraph(world, jointSim, joint);
                    joint.SetIndex = SolverSetType.AwakeSet;
                }
            }

            // transfer island from sleeping set to awake set
            // Usually a sleeping set has only one island, but it is possible
            // that joints are created between sleeping islands and they
            // are moved to the same sleeping set.
            {
                var islands = world.IslandArray;
                int islandCount = set.Islands.Count;
                for (int i = 0; i < islandCount; ++i)
                {
                    IslandSim islandSrc = set.Islands.Data[i];
                    islands.CheckIndex(islandSrc.IslandId);
                    Island island = islands[islandSrc.IslandId];
                    island.SetIndex = SolverSetType.AwakeSet;
                    island.LocalIndex = awakeSet.Islands.Count;
                    IslandSim islandDst = awakeSet.Islands.AddIsland();
                    islandSrc.CopyTo(islandDst);
                }
            }

            // destroy the sleeping set
            DestroySolverSet(world, setIndex);

            SolverSet.ValidateSolverSets(world);
        }

        public static void TrySleepIsland(World world, int islandId)
        {
            world.IslandArray.CheckIndex(islandId);
            Island island = world.IslandArray[islandId];
            Debug.Assert(island.SetIndex == SolverSetType.AwakeSet);

            // cannot put an island to sleep while it has a pending split
            if (island.ConstraintRemoveCount > 0)
            {
                return;
            }

            var moveEvents = world.BodyMoveEventArray;

            // island is sleeping
            // - create new sleeping solver set
            // - move island to sleeping solver set
            // - identify non-touching contacts that should move to sleeping solver set or disabled set
            // - remove old island
            // - fix island
            int sleepSetId = world.SolverSetIdPool.AllocId();
            if (sleepSetId == world.SolverSetArray.Count)
            {
                SolverSet set = new();
                set.SetIndex = Core.NullIndex;
                world.SolverSetArray.Push(set);
            }

            ref SolverSet sleepSet = ref world.SolverSetArray[sleepSetId];
            sleepSet = new SolverSet();

            // grab awake set after creating the sleep set because the solver set array may have been resized
            SolverSet awakeSet = world.SolverSetArray[SolverSetType.AwakeSet];
            Debug.Assert(0 <= island.LocalIndex && island.LocalIndex < awakeSet.Islands.Count);

            sleepSet.SetIndex = sleepSetId;
            sleepSet.Sims = new(island.BodyCount);
            sleepSet.Contacts = new(island.ContactCount);
            sleepSet.Joints = new(island.JointCount);

            // move awake bodies to sleeping set
            // this shuffles around bodies in the awake set
            {
                SolverSet disabledSet = world.SolverSetArray[SolverSetType.DisabledSet];
                var bodies = world.BodyArray;
                var contacts = world.ContactArray;
                int bodyId = island.HeadBody;
                while (bodyId != Core.NullIndex)
                {
                    bodies.CheckIndex(bodyId);
                    Body body = bodies[bodyId];
                    Debug.Assert(body.SetIndex == SolverSetType.AwakeSet);
                    Debug.Assert(body.IslandId == islandId);

                    // Update the body move event to indicate this body fell asleep
                    // It could happen the body is forced asleep before it ever moves.
                    if (body.BodyMoveIndex != Core.NullIndex)
                    {
                        moveEvents.CheckIndex(body.BodyMoveIndex);
                        Debug.Assert(moveEvents[body.BodyMoveIndex].BodyId.Index1 - 1 == bodyId);
                        Debug.Assert(moveEvents[body.BodyMoveIndex].BodyId.Revision == body.Revision);
                        moveEvents[body.BodyMoveIndex].FellAsleep = true;
                        body.BodyMoveIndex = Core.NullIndex;
                    }

                    int awakeBodyIndex = body.LocalIndex;
                    Debug.Assert(0 <= awakeBodyIndex && awakeBodyIndex < awakeSet.Sims.Count);

                    BodySim awakeSim = awakeSet.Sims.Data[awakeBodyIndex];

                    // move body sim to sleep set
                    int sleepBodyIndex = sleepSet.Sims.Count;
                    BodySim sleepBodySim = sleepSet.Sims.AddBodySim();
                    awakeSim.CopyTo(sleepBodySim);

                    int movedIndex = awakeSet.Sims.RemoveBodySim(awakeBodyIndex);
                    if (movedIndex != Core.NullIndex)
                    {
                        // fix local index on moved element
                        BodySim movedSim = awakeSet.Sims.Data[awakeBodyIndex];
                        int movedId = movedSim.BodyId;
                        bodies.CheckIndex(movedId);
                        Body movedBody = bodies[movedId];
                        Debug.Assert(movedBody.LocalIndex == movedIndex);
                        movedBody.LocalIndex = awakeBodyIndex;
                    }

                    // destroy state, no need to clone
                    awakeSet.States.RemoveBodyState(awakeBodyIndex);

                    body.SetIndex = sleepSetId;
                    body.LocalIndex = sleepBodyIndex;

                    // Move non-touching contacts to the disabled set.
                    // Non-touching contacts may exist between sleeping islands and there is no clear ownership.
                    int contactKey = body.HeadContactKey;
                    while (contactKey != Core.NullIndex)
                    {
                        int contactId = contactKey >> 1;
                        int edgeIndex = contactKey & 1;

                        contacts.CheckIndex(contactId);
                        Contact contact = contacts[contactId];

                        Debug.Assert(contact.SetIndex == SolverSetType.AwakeSet || contact.SetIndex == SolverSetType.DisabledSet);
                        contactKey = contact.Edges[edgeIndex].NextKey;

                        if (contact.SetIndex == SolverSetType.DisabledSet)
                        {
                            // already moved to disabled set by another body in the island
                            continue;
                        }

                        if (contact.ColorIndex != Core.NullIndex)
                        {
                            // contact is touching and will be moved separately
                            Debug.Assert(contact.Flags.IsSet(ContactFlags.ContactTouchingFlag));
                            continue;
                        }

                        // the other body may still be awake, it still may go to sleep and then it will be responsible
                        // for moving this contact to the disabled set.
                        int otherEdgeIndex = edgeIndex ^ 1;
                        int otherBodyId = contact.Edges[otherEdgeIndex].BodyId;
                        bodies.CheckIndex(otherBodyId);
                        Body otherBody = bodies[otherBodyId];
                        if (otherBody.SetIndex == SolverSetType.AwakeSet)
                        {
                            continue;
                        }

                        int localIndex = contact.LocalIndex;
                        Debug.Assert(0 <= localIndex && localIndex < awakeSet.Contacts.Count);
                        ContactSim contactSim = awakeSet.Contacts.Data[localIndex];

                        Debug.Assert(contactSim.Manifold.PointCount == 0);
                        Debug.Assert((contact.Flags & ContactFlags.ContactTouchingFlag) == 0 || (contact.Flags & ContactFlags.ContactSensorFlag) != 0);

                        // move the non-touching contact to the disabled set
                        contact.SetIndex = SolverSetType.DisabledSet;
                        contact.LocalIndex = disabledSet.Contacts.Count;
                        ContactSim disabledContactSim = disabledSet.Contacts.AddContact();
                        contactSim.CopyTo(disabledContactSim);

                        int movedContactIndex = awakeSet.Contacts.RemoveContact(localIndex);
                        if (movedContactIndex != Core.NullIndex)
                        {
                            // fix moved element
                            ContactSim movedContactSim = awakeSet.Contacts.Data[localIndex];
                            int movedId = movedContactSim.ContactId;
                            contacts.CheckIndex(movedId);
                            Debug.Assert(contacts[movedId].LocalIndex == movedContactIndex);
                            contacts[movedId].LocalIndex = localIndex;
                        }
                    }

                    bodyId = body.IslandNext;
                }
            }

            // move touching contacts
            // this shuffles contacts in the awake set
            {
                var contacts = world.ContactArray;
                int contactId = island.HeadContact;
                while (contactId != Core.NullIndex)
                {
                    contacts.CheckIndex(contactId);
                    Contact contact = contacts[contactId];
                    Debug.Assert(contact.SetIndex == SolverSetType.AwakeSet);
                    Debug.Assert(contact.IslandId == islandId);
                    int colorIndex = contact.ColorIndex;
                    Debug.Assert(0 <= colorIndex && colorIndex < Core.GraphColorCount);

                    GraphColor color = world.ConstraintGraph.Colors[colorIndex];

                    // Remove bodies from graph coloring associated with this constraint
                    if (colorIndex != Core.OverflowIndex)
                    {
                        // might clear a bit for a static body, but this has no effect
                        color.BodySet.ClearBit(contact.Edges[0].BodyId);
                        color.BodySet.ClearBit(contact.Edges[1].BodyId);
                    }

                    int awakeContactIndex = contact.LocalIndex;
                    Debug.Assert(0 <= awakeContactIndex && awakeContactIndex < color.Contacts.Count);
                    ContactSim awakeContactSim = color.Contacts[awakeContactIndex];

                    int sleepContactIndex = sleepSet.Contacts.Count;
                    ContactSim sleepContactSim = sleepSet.Contacts.AddContact();
                    awakeContactSim.CopyTo(sleepContactSim);

                    int movedIndex = color.Contacts.RemoveContact(awakeContactIndex);
                    if (movedIndex != Core.NullIndex)
                    {
                        // fix moved element
                        var movedContactSim = color.Contacts[awakeContactIndex];
                        int movedId = movedContactSim.ContactId;
                        contacts.CheckIndex(movedId);
                        Contact movedContact = contacts[movedId];
                        Debug.Assert(movedContact.LocalIndex == movedIndex);
                        movedContact.LocalIndex = awakeContactIndex;
                    }

                    contact.SetIndex = sleepSetId;
                    contact.ColorIndex = Core.NullIndex;
                    contact.LocalIndex = sleepContactIndex;

                    contactId = contact.IslandNext;
                }
            }

            // move joints
            // this shuffles joints in the awake set
            {
                var joints = world.JointArray;
                int jointId = island.HeadJoint;
                while (jointId != Core.NullIndex)
                {
                    joints.CheckIndex(jointId);
                    Joint joint = joints[jointId];
                    Debug.Assert(joint.SetIndex == SolverSetType.AwakeSet);
                    Debug.Assert(joint.IslandId == islandId);
                    int colorIndex = joint.ColorIndex;
                    int localIndex = joint.LocalIndex;

                    Debug.Assert(0 <= colorIndex && colorIndex < Core.GraphColorCount);

                    var color = world.ConstraintGraph.Colors[colorIndex];

                    Debug.Assert(0 <= localIndex && localIndex < color.Joints.Count);
                    JointSim awakeJointSim = color.Joints[localIndex];

                    if (colorIndex != Core.OverflowIndex)
                    {
                        // might clear a bit for a static body, but this has no effect
                        color.BodySet.ClearBit(joint.Edges[0].BodyId);
                        color.BodySet.ClearBit(joint.Edges[1].BodyId);
                    }

                    int sleepJointIndex = sleepSet.Joints.Count;
                    JointSim sleepJointSim = sleepSet.Joints.AddJoint();
                    awakeJointSim.CopyTo(sleepJointSim);

                    int movedIndex = color.Joints.RemoveJoint(localIndex);
                    if (movedIndex != Core.NullIndex)
                    {
                        // fix moved element
                        JointSim movedJointSim = color.Joints[localIndex];
                        int movedId = movedJointSim.JointId;
                        joints.CheckIndex(movedId);
                        Joint movedJoint = joints[movedId];
                        Debug.Assert(movedJoint.LocalIndex == movedIndex);
                        movedJoint.LocalIndex = localIndex;
                    }

                    joint.SetIndex = sleepSetId;
                    joint.ColorIndex = Core.NullIndex;
                    joint.LocalIndex = sleepJointIndex;

                    jointId = joint.IslandNext;
                }
            }

            // move island struct
            {
                Debug.Assert(island.SetIndex == SolverSetType.AwakeSet);

                int islandIndex = island.LocalIndex;
                IslandSim sleepIsland = sleepSet.Islands.AddIsland();
                sleepIsland.IslandId = islandId;

                int movedIslandIndex = awakeSet.Islands.RemoveIsland(islandIndex);
                if (movedIslandIndex != Core.NullIndex)
                {
                    // fix index on moved element
                    IslandSim movedIslandSim = awakeSet.Islands[islandIndex];
                    int movedIslandId = movedIslandSim.IslandId;
                    world.IslandArray.CheckIndex(movedIslandId);
                    Island movedIsland = world.IslandArray[movedIslandId];
                    Debug.Assert(movedIsland.LocalIndex == movedIslandIndex);
                    movedIsland.LocalIndex = islandIndex;
                }

                island.SetIndex = sleepSetId;
                island.LocalIndex = 0;
            }

            ValidateSolverSets(world);
        }

        // This is called when joints are created between sets. I want to allow the sets
        // to continue sleeping if both are asleep. Otherwise one set is waked.
        // Islands will get merge when the set is waked.
        public static void MergeSolverSets(World world, int setId1, int setId2)
        {
            Debug.Assert(setId1 >= SolverSetType.FirstSleepingSet);
            Debug.Assert(setId2 >= SolverSetType.FirstSleepingSet);
            world.SolverSetArray.CheckIndex(setId1);
            world.SolverSetArray.CheckIndex(setId2);
            SolverSet set1 = world.SolverSetArray[setId1];
            SolverSet set2 = world.SolverSetArray[setId2];

            // Move the fewest number of bodies
            if (set1.Sims.Count < set2.Sims.Count)
            {
                (set1, set2) = (set2, set1);

                (setId1, setId2) = (setId2, setId1);
            }

            // transfer bodies
            {
                var bodies = world.BodyArray;
                int bodyCount = set2.Sims.Count;
                for (int i = 0; i < bodyCount; ++i)
                {
                    BodySim simSrc = set2.Sims.Data[i];

                    Body body = bodies[simSrc.BodyId];
                    Debug.Assert(body.SetIndex == setId2);
                    body.SetIndex = setId1;
                    body.LocalIndex = set1.Sims.Count;

                    BodySim simDst = set1.Sims.AddBodySim();
                    simSrc.CopyTo(simDst);
                }
            }

            // transfer contacts
            {
                var contacts = world.ContactArray;
                int contactCount = set2.Contacts.Count;
                for (int i = 0; i < contactCount; ++i)
                {
                    ContactSim contactSrc = set2.Contacts.Data[i];

                    Contact contact = contacts[contactSrc.ContactId];
                    Debug.Assert(contact.SetIndex == setId2);
                    contact.SetIndex = setId1;
                    contact.LocalIndex = set1.Contacts.Count;

                    ContactSim contactDst = set1.Contacts.AddContact();
                    contactSrc.CopyTo(contactDst);
                }
            }

            // transfer joints
            {
                var joints = world.JointArray;
                int jointCount = set2.Joints.Count;
                for (int i = 0; i < jointCount; ++i)
                {
                    JointSim jointSrc = set2.Joints.Data[i];

                    Joint joint = joints[jointSrc.JointId];
                    Debug.Assert(joint.SetIndex == setId2);
                    joint.SetIndex = setId1;
                    joint.LocalIndex = set1.Joints.Count;

                    JointSim jointDst = set1.Joints.AddJoint();
                    jointSrc.CopyTo(jointDst);
                }
            }

            // transfer islands
            {
                var islands = world.IslandArray;
                int islandCount = set2.Islands.Count;
                for (int i = 0; i < islandCount; ++i)
                {
                    IslandSim islandSrc = set2.Islands.Data[i];
                    int islandId = islandSrc.IslandId;

                    islands.CheckIndex(islandId);
                    Island island = islands[islandId];
                    island.SetIndex = setId1;
                    island.LocalIndex = set1.Islands.Count;

                    IslandSim islandDst = set1.Islands.AddIsland();
                    islandSrc.CopyTo(islandDst);
                }
            }

            // destroy the merged set
            DestroySolverSet(world, setId2);

            SolverSet.ValidateSolverSets(world);
        }

        public static void TransferBody(World world, SolverSet targetSet, SolverSet sourceSet, Body body)
        {
            Debug.Assert(targetSet != sourceSet);

            int sourceIndex = body.LocalIndex;
            Debug.Assert(0 <= sourceIndex && sourceIndex <= sourceSet.Sims.Count);
            BodySim sourceSim = sourceSet.Sims.Data[sourceIndex];

            int targetIndex = targetSet.Sims.Count;
            BodySim targetSim = targetSet.Sims.AddBodySim();
            sourceSim.CopyTo(targetSim);

            // Remove body sim from solver set that owns it
            int movedIndex = sourceSet.Sims.RemoveBodySim(sourceIndex);
            if (movedIndex != Core.NullIndex)
            {
                // Fix moved body index
                BodySim movedSim = sourceSet.Sims.Data[sourceIndex];
                int movedId = movedSim.BodyId;
                Body movedBody = world.BodyArray[movedId];
                Debug.Assert(movedBody.LocalIndex == movedIndex);
                movedBody.LocalIndex = sourceIndex;
            }

            if (sourceSet.SetIndex == SolverSetType.AwakeSet)
            {
                sourceSet.States.RemoveBodyState(sourceIndex);
            }
            else if (targetSet.SetIndex == SolverSetType.AwakeSet)
            {
                ref BodyState state = ref targetSet.States.AddBodyState();
                state = BodyState.Identity;
            }

            body.SetIndex = targetSet.SetIndex;
            body.LocalIndex = targetIndex;
        }

        public static void TransferJoint(World world, SolverSet targetSet, SolverSet sourceSet, Joint joint)
        {
            Debug.Assert(targetSet != sourceSet);

            int localIndex = joint.LocalIndex;
            int colorIndex = joint.ColorIndex;

            // Retrieve source.
            JointSim sourceSim;
            if (sourceSet.SetIndex == SolverSetType.AwakeSet)
            {
                Debug.Assert(colorIndex is >= 0 and < Core.GraphColorCount);
                GraphColor color = world.ConstraintGraph.Colors[colorIndex];

                Debug.Assert(0 <= localIndex && localIndex < color.Joints.Count);
                sourceSim = color.Joints.Data[localIndex];
            }
            else
            {
                Debug.Assert(colorIndex == Core.NullIndex);
                Debug.Assert(0 <= localIndex && localIndex < sourceSet.Joints.Count);
                sourceSim = sourceSet.Joints.Data[localIndex];
            }

            // Create target and copy. Fix joint.
            if (targetSet.SetIndex == SolverSetType.AwakeSet)
            {
                ConstraintGraph.AddJointToGraph(world, sourceSim, joint);
                joint.SetIndex = SolverSetType.AwakeSet;
            }
            else
            {
                joint.SetIndex = targetSet.SetIndex;
                joint.LocalIndex = targetSet.Joints.Count;
                joint.ColorIndex = Core.NullIndex;

                JointSim targetSim = targetSet.Joints.AddJoint();
                sourceSim.CopyTo(targetSim);
            }

            // Destroy source.
            if (sourceSet.SetIndex == SolverSetType.AwakeSet)
            {
                ConstraintGraph.RemoveJointFromGraph(world, joint.Edges[0].BodyId, joint.Edges[1].BodyId, colorIndex, localIndex);
            }
            else
            {
                int movedIndex = sourceSet.Joints.RemoveJoint(localIndex);
                if (movedIndex != Core.NullIndex)
                {
                    // fix swapped element
                    JointSim movedJointSim = sourceSet.Joints.Data[localIndex];
                    int movedId = movedJointSim.JointId;
                    world.JointArray.CheckIndex(movedId);
                    Joint movedJoint = world.JointArray[movedId];
                    movedJoint.LocalIndex = localIndex;
                }
            }
        }
    }
}