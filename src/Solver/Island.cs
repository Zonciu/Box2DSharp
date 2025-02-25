using System;
using System.Diagnostics;

namespace Box2DSharp
{
    // Deterministic solver
    //
    // Collide all awake contacts
    // Use bit array to emit start/stop touching events in defined order, per thread. Try using contact index, assuming contacts are
    // created in a deterministic order. bit-wise OR together bit arrays and issue changes:
    // - start touching: merge islands - temporary linked list - mark root island dirty - wake all - largest island is root
    // - stop touching: mark island dirty - wake island
    // Reserve island jobs
    // - island job does a DFS to merge/split islands. Mutex to allocate new islands. Split islands sent to different jobs.

    // Persistent island for awake bodies, joints, and contacts
    // https://en.wikipedia.org/wiki/Component_(graph_theory)
    // https://en.wikipedia.org/wiki/Dynamic_connectivity
    // map from int to solver set and index
    public class Island
    {
        // index of solver set stored in b2World
        // may be Core.B2_NULL_INDEX
        public int SetIndex;

        // island index within set
        // may be Core.B2_NULL_INDEX
        public int LocalIndex;

        public int IslandId;

        public int HeadBody;

        public int TailBody;

        public int BodyCount;

        public int HeadContact;

        public int TailContact;

        public int ContactCount;

        public int HeadJoint;

        public int TailJoint;

        public int JointCount;

        // Union find
        public int ParentIsland;

        // Keeps track of how many contacts have been removed from this island.
        public int ConstraintRemoveCount;

        public static Island CreateIsland(World world, int setIndex)
        {
            Debug.Assert(setIndex == SolverSetType.AwakeSet || setIndex >= SolverSetType.FirstSleepingSet);

            int islandId = world.IslandIdPool.AllocId();

            if (islandId == world.IslandArray.Count)
            {
                Island emptyIsland = new();
                world.IslandArray.Push(emptyIsland);
            }
            else
            {
                Debug.Assert(world.IslandArray[islandId].SetIndex == Core.NullIndex);
            }

            world.SolverSetArray.CheckIndex(setIndex);
            ref SolverSet set = ref world.SolverSetArray[setIndex];

            Island island = world.IslandArray[islandId];
            island.SetIndex = setIndex;
            island.LocalIndex = set.Islands.Count;
            island.IslandId = islandId;
            island.HeadBody = Core.NullIndex;
            island.TailBody = Core.NullIndex;
            island.BodyCount = 0;
            island.HeadContact = Core.NullIndex;
            island.TailContact = Core.NullIndex;
            island.ContactCount = 0;
            island.HeadJoint = Core.NullIndex;
            island.TailJoint = Core.NullIndex;
            island.JointCount = 0;
            island.ParentIsland = Core.NullIndex;
            island.ConstraintRemoveCount = 0;

            IslandSim islandSim = set.Islands.AddIsland();
            islandSim.IslandId = islandId;

            return island;
        }

        public static void DestroyIsland(World world, int islandId)
        {
            // assume island is empty
            world.IslandArray.CheckIndex(islandId);
            Island island = world.IslandArray[islandId];
            world.SolverSetArray.CheckIndex(island.SetIndex);
            ref SolverSet set = ref world.SolverSetArray[island.SetIndex];
            int movedIndex = set.Islands.RemoveIsland(island.LocalIndex);
            if (movedIndex != Core.NullIndex)
            {
                // Fix index on moved element
                IslandSim movedElement = set.Islands.Data[island.LocalIndex];
                int movedId = movedElement.IslandId;
                Island movedIsland = world.IslandArray[movedId];
                Debug.Assert(movedIsland.LocalIndex == movedIndex);
                movedIsland.LocalIndex = island.LocalIndex;
            }

            // Free island and id (preserve island revision)
            island.IslandId = Core.NullIndex;
            island.SetIndex = Core.NullIndex;
            island.LocalIndex = Core.NullIndex;
            world.IslandIdPool.FreeId(islandId);
        }

        public static Island GetIsland(World world, int islandId)
        {
            world.IslandArray.CheckIndex(islandId);
            return world.IslandArray[islandId];
        }

        public static void ValidateIsland(World world, int islandId)
        {
            if (!Core.B2Validate)
            {
                return;
            }

            world.IslandArray.CheckIndex(islandId);
            Island island = world.IslandArray[islandId];
            Debug.Assert(island.IslandId == islandId);
            Debug.Assert(island.SetIndex != Core.NullIndex);
            Debug.Assert(island.HeadBody != Core.NullIndex);

            {
                var bodies = world.BodyArray;
                Debug.Assert(island.TailBody != Core.NullIndex);
                Debug.Assert(island.BodyCount > 0);
                if (island.BodyCount > 1)
                {
                    Debug.Assert(island.TailBody != island.HeadBody);
                }

                Debug.Assert(island.BodyCount <= world.BodyIdPool.GetIdCount());

                int count = 0;
                int bodyId = island.HeadBody;
                while (bodyId != Core.NullIndex)
                {
                    bodies.CheckIndex(bodyId);
                    Body body = bodies[bodyId];
                    Debug.Assert(body.IslandId == islandId);
                    Debug.Assert(body.SetIndex == island.SetIndex);
                    count += 1;

                    if (count == island.BodyCount)
                    {
                        Debug.Assert(bodyId == island.TailBody);
                    }

                    bodyId = body.IslandNext;
                }

                Debug.Assert(count == island.BodyCount);
            }

            if (island.HeadContact != Core.NullIndex)
            {
                Debug.Assert(island.TailContact != Core.NullIndex);
                Debug.Assert(island.ContactCount > 0);
                if (island.ContactCount > 1)
                {
                    Debug.Assert(island.TailContact != island.HeadContact);
                }

                Debug.Assert(island.ContactCount <= world.ContactIdPool.GetIdCount());

                int count = 0;
                int contactId = island.HeadContact;
                while (contactId != Core.NullIndex)
                {
                    world.ContactArray.CheckIndex(contactId);
                    ref Contact contact = ref world.ContactArray[contactId];
                    Debug.Assert(contact.SetIndex == island.SetIndex);
                    Debug.Assert(contact.IslandId == islandId);
                    count += 1;

                    if (count == island.ContactCount)
                    {
                        Debug.Assert(contactId == island.TailContact);
                    }

                    contactId = contact.IslandNext;
                }

                Debug.Assert(count == island.ContactCount);
            }
            else
            {
                Debug.Assert(island.TailContact == Core.NullIndex);
                Debug.Assert(island.ContactCount == 0);
            }

            if (island.HeadJoint != Core.NullIndex)
            {
                Debug.Assert(island.TailJoint != Core.NullIndex);
                Debug.Assert(island.JointCount > 0);
                if (island.JointCount > 1)
                {
                    Debug.Assert(island.TailJoint != island.HeadJoint);
                }

                Debug.Assert(island.JointCount <= world.JointIdPool.GetIdCount());

                int count = 0;
                int jointId = island.HeadJoint;
                while (jointId != Core.NullIndex)
                {
                    world.JointArray.CheckIndex(jointId);
                    ref Joint joint = ref world.JointArray[jointId];
                    Debug.Assert(joint.SetIndex == island.SetIndex);
                    count += 1;

                    if (count == island.JointCount)
                    {
                        Debug.Assert(jointId == island.TailJoint);
                    }

                    jointId = joint.IslandNext;
                }

                Debug.Assert(count == island.JointCount);
            }
            else
            {
                Debug.Assert(island.TailJoint == Core.NullIndex);
                Debug.Assert(island.JointCount == 0);
            }
        }

        private static void AddContactToIsland(World world, int islandId, ref Contact contact)
        {
            Debug.Assert(contact.IslandId == Core.NullIndex);
            Debug.Assert(contact.IslandPrev == Core.NullIndex);
            Debug.Assert(contact.IslandNext == Core.NullIndex);

            world.IslandArray.CheckIndex(islandId);
            Island island = world.IslandArray[islandId];

            if (island.HeadContact != Core.NullIndex)
            {
                contact.IslandNext = island.HeadContact;
                world.ContactArray.CheckIndex(island.HeadContact);
                ref Contact headContact = ref world.ContactArray[island.HeadContact];
                headContact.IslandPrev = contact.ContactId;
            }

            island.HeadContact = contact.ContactId;
            if (island.TailContact == Core.NullIndex)
            {
                island.TailContact = island.HeadContact;
            }

            island.ContactCount += 1;
            contact.IslandId = islandId;

            ValidateIsland(world, islandId);
        }

        // Link a contact into an island.
        // This performs union-find and path compression to join islands.
        // https://en.wikipedia.org/wiki/Disjoint-set_data_structure
        public static void LinkContact(World world, ref Contact contact)
        {
            Debug.Assert((contact.Flags & ContactFlags.ContactTouchingFlag) != 0 && (contact.Flags & ContactFlags.ContactSensorFlag) == 0);

            int bodyIdA = contact.Edges[0].BodyId;
            int bodyIdB = contact.Edges[1].BodyId;

            Body bodyA = Body.GetBody(world, bodyIdA);
            Body bodyB = Body.GetBody(world, bodyIdB);

            Debug.Assert(bodyA.SetIndex != SolverSetType.DisabledSet && bodyB.SetIndex != SolverSetType.DisabledSet);
            Debug.Assert(bodyA.SetIndex != SolverSetType.StaticSet || bodyB.SetIndex != SolverSetType.StaticSet);

            // Wake bodyB if bodyA is awake and bodyB is sleeping
            if (bodyA.SetIndex == SolverSetType.AwakeSet && bodyB.SetIndex >= SolverSetType.FirstSleepingSet)
            {
                SolverSet.WakeSolverSet(world, bodyB.SetIndex);
            }

            // Wake bodyA if bodyB is awake and bodyA is sleeping
            if (bodyB.SetIndex == SolverSetType.AwakeSet && bodyA.SetIndex >= SolverSetType.FirstSleepingSet)
            {
                SolverSet.WakeSolverSet(world, bodyA.SetIndex);
            }

            int islandIdA = bodyA.IslandId;
            int islandIdB = bodyB.IslandId;

            // Static bodies have null island indices.
            Debug.Assert(bodyA.SetIndex != SolverSetType.StaticSet || islandIdA == Core.NullIndex);
            Debug.Assert(bodyB.SetIndex != SolverSetType.StaticSet || islandIdB == Core.NullIndex);
            Debug.Assert(islandIdA != Core.NullIndex || islandIdB != Core.NullIndex);

            if (islandIdA == islandIdB)
            {
                // Contact in same island
                AddContactToIsland(world, islandIdA, ref contact);
                return;
            }

            // Union-find root of islandA
            Island? islandA = null;
            if (islandIdA != Core.NullIndex)
            {
                islandA = GetIsland(world, islandIdA);
                int parentId = islandA.ParentIsland;
                while (parentId != Core.NullIndex)
                {
                    Island parent = GetIsland(world, parentId);
                    if (parent.ParentIsland != Core.NullIndex)
                    {
                        // path compression
                        islandA.ParentIsland = parent.ParentIsland;
                    }

                    islandA = parent;
                    islandIdA = parentId;
                    parentId = islandA.ParentIsland;
                }
            }

            // Union-find root of islandB
            Island? islandB = null;
            if (islandIdB != Core.NullIndex)
            {
                islandB = GetIsland(world, islandIdB);
                int parentId = islandB.ParentIsland;
                while (islandB.ParentIsland != Core.NullIndex)
                {
                    Island parent = GetIsland(world, parentId);
                    if (parent.ParentIsland != Core.NullIndex)
                    {
                        // path compression
                        islandB.ParentIsland = parent.ParentIsland;
                    }

                    islandB = parent;
                    islandIdB = parentId;
                    parentId = islandB.ParentIsland;
                }
            }

            Debug.Assert(islandA != null || islandB != null);

            // Union-Find link island roots
            if (islandA != islandB && islandA != null && islandB != null)
            {
                Debug.Assert(islandA != islandB);
                Debug.Assert(islandB.ParentIsland == Core.NullIndex);
                islandB.ParentIsland = islandIdA;
            }

            if (islandA != null)
            {
                AddContactToIsland(world, islandIdA, ref contact);
            }
            else
            {
                AddContactToIsland(world, islandIdB, ref contact);
            }
        }

        // This is called when a contact no longer has contact points or when a contact is destroyed.
        public static void UnlinkContact(World world, Contact contact)
        {
            Debug.Assert((contact.Flags & ContactFlags.ContactSensorFlag) == 0);
            Debug.Assert(contact.IslandId != Core.NullIndex);

            // remove from island
            int islandId = contact.IslandId;
            world.IslandArray.CheckIndex(islandId);
            Island island = GetIsland(world, islandId);

            if (contact.IslandPrev != Core.NullIndex)
            {
                world.ContactArray.CheckIndex(contact.IslandPrev);
                ref Contact prevContact = ref world.ContactArray[contact.IslandPrev];
                Debug.Assert(prevContact.IslandNext == contact.ContactId);
                prevContact.IslandNext = contact.IslandNext;
            }

            if (contact.IslandNext != Core.NullIndex)
            {
                world.ContactArray.CheckIndex(contact.IslandNext);
                ref Contact nextContact = ref world.ContactArray[contact.IslandNext];
                Debug.Assert(nextContact.IslandPrev == contact.ContactId);
                nextContact.IslandPrev = contact.IslandPrev;
            }

            if (island.HeadContact == contact.ContactId)
            {
                island.HeadContact = contact.IslandNext;
            }

            if (island.TailContact == contact.ContactId)
            {
                island.TailContact = contact.IslandPrev;
            }

            Debug.Assert(island.ContactCount > 0);
            island.ContactCount -= 1;
            island.ConstraintRemoveCount += 1;

            contact.IslandId = Core.NullIndex;
            contact.IslandPrev = Core.NullIndex;
            contact.IslandNext = Core.NullIndex;

            ValidateIsland(world, islandId);
        }

        public static void AddJointToIsland(World world, int islandId, Joint joint)
        {
            Debug.Assert(joint.IslandId == Core.NullIndex);
            Debug.Assert(joint.IslandPrev == Core.NullIndex);
            Debug.Assert(joint.IslandNext == Core.NullIndex);

            world.IslandArray.CheckIndex(islandId);
            Island island = world.IslandArray[islandId];

            if (island.HeadJoint != Core.NullIndex)
            {
                joint.IslandNext = island.HeadJoint;
                Joint headJoint = Joint.GetJoint(world, island.HeadJoint);
                headJoint.IslandPrev = joint.JointId;
            }

            island.HeadJoint = joint.JointId;
            if (island.TailJoint == Core.NullIndex)
            {
                island.TailJoint = island.HeadJoint;
            }

            island.JointCount += 1;
            joint.IslandId = islandId;

            ValidateIsland(world, islandId);
        }

        public static void LinkJoint(World world, Joint joint, bool mergeIslands)
        {
            Body bodyA = Body.GetBody(world, joint.Edges[0].BodyId);
            Body bodyB = Body.GetBody(world, joint.Edges[1].BodyId);

            if (bodyA.SetIndex == SolverSetType.AwakeSet && bodyB.SetIndex >= SolverSetType.FirstSleepingSet)
            {
                SolverSet.WakeSolverSet(world, bodyB.SetIndex);
            }
            else if (bodyB.SetIndex == SolverSetType.AwakeSet && bodyA.SetIndex >= SolverSetType.FirstSleepingSet)
            {
                SolverSet.WakeSolverSet(world, bodyA.SetIndex);
            }

            int islandIdA = bodyA.IslandId;
            int islandIdB = bodyB.IslandId;

            Debug.Assert(islandIdA != Core.NullIndex || islandIdB != Core.NullIndex);

            if (islandIdA == islandIdB)
            {
                // Joint in same island
                AddJointToIsland(world, islandIdA, joint);
                return;
            }

            // Union-find root of islandA
            Island islandA = null!;
            if (islandIdA != Core.NullIndex)
            {
                islandA = GetIsland(world, islandIdA);
                while (islandA.ParentIsland != Core.NullIndex)
                {
                    Island parent = GetIsland(world, islandA.ParentIsland);
                    if (parent.ParentIsland != Core.NullIndex)
                    {
                        // path compression
                        islandA.ParentIsland = parent.ParentIsland;
                    }

                    islandIdA = islandA.ParentIsland;
                    islandA = parent;
                }
            }

            // Union-find root of islandB
            Island islandB = null;
            if (islandIdB != Core.NullIndex)
            {
                islandB = GetIsland(world, islandIdB);
                while (islandB.ParentIsland != Core.NullIndex)
                {
                    Island parent = GetIsland(world, islandB.ParentIsland);
                    if (parent.ParentIsland != Core.NullIndex)
                    {
                        // path compression
                        islandB.ParentIsland = parent.ParentIsland;
                    }

                    islandIdB = islandB.ParentIsland;
                    islandB = parent;
                }
            }

            Debug.Assert(islandA != null || islandB != null);

            // Union-Find link island roots
            if (islandA != islandB && islandA != null && islandB != null)
            {
                Debug.Assert(islandA != islandB);
                Debug.Assert(islandB.ParentIsland == Core.NullIndex);
                islandB.ParentIsland = islandIdA;
            }

            if (islandA != null)
            {
                AddJointToIsland(world, islandIdA, joint);
            }
            else
            {
                AddJointToIsland(world, islandIdB, joint);
            }

            // Joints need to have islands merged immediately when they are created
            // to keep the island graph valid.
            // However, when a body type is being changed the merge can be deferred until
            // all joints are linked.
            if (mergeIslands)
            {
                MergeAwakeIslands(world);
            }
        }

        public static void UnlinkJoint(World world, Joint joint)
        {
            Debug.Assert(joint.IslandId != Core.NullIndex);

            // remove from island
            int islandId = joint.IslandId;
            world.IslandArray.CheckIndex(islandId);
            Island island = world.IslandArray[islandId];

            if (joint.IslandPrev != Core.NullIndex)
            {
                Joint prevJoint = Joint.GetJoint(world, joint.IslandPrev);
                Debug.Assert(prevJoint.IslandNext == joint.JointId);
                prevJoint.IslandNext = joint.IslandNext;
            }

            if (joint.IslandNext != Core.NullIndex)
            {
                Joint nextJoint = Joint.GetJoint(world, joint.IslandNext);
                Debug.Assert(nextJoint.IslandPrev == joint.JointId);
                nextJoint.IslandPrev = joint.IslandPrev;
            }

            if (island.HeadJoint == joint.JointId)
            {
                island.HeadJoint = joint.IslandNext;
            }

            if (island.TailJoint == joint.JointId)
            {
                island.TailJoint = joint.IslandPrev;
            }

            Debug.Assert(island.JointCount > 0);
            island.JointCount -= 1;
            island.ConstraintRemoveCount += 1;

            joint.IslandId = Core.NullIndex;
            joint.IslandPrev = Core.NullIndex;
            joint.IslandNext = Core.NullIndex;

            ValidateIsland(world, islandId);
        }

        // Merge an island into its root island.
        // todo we can assume all islands are awake here
        public static void MergeIsland(World world, Island island)
        {
            Debug.Assert(island.ParentIsland != Core.NullIndex);

            int rootId = island.ParentIsland;
            world.IslandArray.CheckIndex(rootId);
            Island rootIsland = world.IslandArray[rootId];
            Debug.Assert(rootIsland.ParentIsland == Core.NullIndex);

            // remap island indices
            int bodyId = island.HeadBody;
            while (bodyId != Core.NullIndex)
            {
                Body body = Body.GetBody(world, bodyId);
                body.IslandId = rootId;
                bodyId = body.IslandNext;
            }

            int contactId = island.HeadContact;
            while (contactId != Core.NullIndex)
            {
                world.ContactArray.CheckIndex(contactId);
                Contact contact = world.ContactArray[contactId];
                contact.IslandId = rootId;
                contactId = contact.IslandNext;
            }

            int jointId = island.HeadJoint;
            while (jointId != Core.NullIndex)
            {
                Joint joint = Joint.GetJoint(world, jointId);
                joint.IslandId = rootId;
                jointId = joint.IslandNext;
            }

            // connect body lists
            Debug.Assert(rootIsland.TailBody != Core.NullIndex);
            Body tailBody = Body.GetBody(world, rootIsland.TailBody);
            Debug.Assert(tailBody.IslandNext == Core.NullIndex);
            tailBody.IslandNext = island.HeadBody;

            Debug.Assert(island.HeadBody != Core.NullIndex);
            Body headBody = Body.GetBody(world, island.HeadBody);
            Debug.Assert(headBody.IslandPrev == Core.NullIndex);
            headBody.IslandPrev = rootIsland.TailBody;

            rootIsland.TailBody = island.TailBody;
            rootIsland.BodyCount += island.BodyCount;

            // connect contact lists
            if (rootIsland.HeadContact == Core.NullIndex)
            {
                // Root island has no contacts
                Debug.Assert(rootIsland.TailContact == Core.NullIndex && rootIsland.ContactCount == 0);
                rootIsland.HeadContact = island.HeadContact;
                rootIsland.TailContact = island.TailContact;
                rootIsland.ContactCount = island.ContactCount;
            }
            else if (island.HeadContact != Core.NullIndex)
            {
                // Both islands have contacts
                Debug.Assert(island.TailContact != Core.NullIndex && island.ContactCount > 0);
                Debug.Assert(rootIsland.TailContact != Core.NullIndex && rootIsland.ContactCount > 0);

                world.ContactArray.CheckIndex(rootIsland.TailContact);
                Contact tailContact = world.ContactArray[rootIsland.TailContact];
                Debug.Assert(tailContact.IslandNext == Core.NullIndex);
                tailContact.IslandNext = island.HeadContact;

                world.ContactArray.CheckIndex(island.HeadContact);
                Contact headContact = world.ContactArray[island.HeadContact];
                Debug.Assert(headContact.IslandPrev == Core.NullIndex);
                headContact.IslandPrev = rootIsland.TailContact;

                rootIsland.TailContact = island.TailContact;
                rootIsland.ContactCount += island.ContactCount;
            }

            if (rootIsland.HeadJoint == Core.NullIndex)
            {
                // Root island has no joints
                Debug.Assert(rootIsland.TailJoint == Core.NullIndex && rootIsland.JointCount == 0);
                rootIsland.HeadJoint = island.HeadJoint;
                rootIsland.TailJoint = island.TailJoint;
                rootIsland.JointCount = island.JointCount;
            }
            else if (island.HeadJoint != Core.NullIndex)
            {
                // Both islands have joints
                Debug.Assert(island.TailJoint != Core.NullIndex && island.JointCount > 0);
                Debug.Assert(rootIsland.TailJoint != Core.NullIndex && rootIsland.JointCount > 0);

                Joint tailJoint = Joint.GetJoint(world, rootIsland.TailJoint);
                Debug.Assert(tailJoint.IslandNext == Core.NullIndex);
                tailJoint.IslandNext = island.HeadJoint;

                Joint headJoint = Joint.GetJoint(world, island.HeadJoint);
                Debug.Assert(headJoint.IslandPrev == Core.NullIndex);
                headJoint.IslandPrev = rootIsland.TailJoint;

                rootIsland.TailJoint = island.TailJoint;
                rootIsland.JointCount += island.JointCount;
            }

            // Track removed constraints
            rootIsland.ConstraintRemoveCount += island.ConstraintRemoveCount;

            ValidateIsland(world, rootId);
        }

        /// <summary>
        /// Iterate over all awake islands and merge any that need merging
        /// Islands that get merged into a root island will be removed from the awake island array
        /// and returned to the pool.
        /// todo this might be faster if b2IslandSim held the connectivity data
        /// </summary>
        /// <param name="world"></param>
        public static void MergeAwakeIslands(World world)
        {
            SolverSet awakeSet = world.SolverSetArray[SolverSetType.AwakeSet];
            Span<IslandSim> islandSims = awakeSet.Islands;
            int awakeIslandCount = awakeSet.Islands.Count;
            var islands = world.IslandArray;

            // Step 1: Ensure every child island points to its root island. This avoids merging a child island with
            // a parent island that has already been merged with a grand-parent island.
            for (int i = 0; i < awakeIslandCount; ++i)
            {
                int islandId = islandSims[i].IslandId;

                islands.CheckIndex(islandId);
                Island island = islands[islandId];

                // find the root island
                int rootId = islandId;
                Island rootIsland = island;
                while (rootIsland.ParentIsland != Core.NullIndex)
                {
                    islands.CheckIndex(rootIsland.ParentIsland);
                    Island parent = islands[rootIsland.ParentIsland];
                    if (parent.ParentIsland != Core.NullIndex)
                    {
                        // path compression
                        rootIsland.ParentIsland = parent.ParentIsland;
                    }

                    rootId = rootIsland.ParentIsland;
                    rootIsland = parent;
                }

                if (rootIsland != island)
                {
                    island.ParentIsland = rootId;
                }
            }

            // Step 2: merge every awake island into its parent (which must be a root island)
            // Reverse to support removal from awake array.
            for (int i = awakeIslandCount - 1; i >= 0; --i)
            {
                int islandId = islandSims[i].IslandId;
                islands.CheckIndex(islandId);
                Island island = islands[islandId];

                if (island.ParentIsland == Core.NullIndex)
                {
                    continue;
                }

                MergeIsland(world, island);

                // this call does a remove swap from the end of the island sim array
                DestroyIsland(world, islandId);
            }

            SolverSet.ValidateConnectivity(world);
        }

        public static void SplitIsland(World world, int baseId)
        {
            world.IslandArray.CheckIndex(baseId);
            Island baseIsland = world.IslandArray[baseId];
            int setIndex = baseIsland.SetIndex;

            if (setIndex != SolverSetType.AwakeSet)
            {
                // can only split awake island
                return;
            }

            if (baseIsland.ConstraintRemoveCount == 0)
            {
                // this island doesn't need to be split
                return;
            }

            ValidateIsland(world, baseId);

            int bodyCount = baseIsland.BodyCount;

            var bodies = world.BodyArray;
            var contacts = world.ContactArray;

            // No lock is needed because I ensure the allocator is not used while this task is active.
            int[] stackArray = B2ArrayPool<int>.Shared.Rent(bodyCount);
            int[] bodyIdsArray = B2ArrayPool<int>.Shared.Rent(bodyCount);
            Span<int> stack = stackArray.AsSpan();
            Span<int> bodyIds = bodyIdsArray.AsSpan();

            // Build array containing all body indices from base island. These
            // serve as seed bodies for the depth first search (DFS).
            int index = 0;
            int nextBody = baseIsland.HeadBody;
            while (nextBody != Core.NullIndex)
            {
                bodyIds[index++] = nextBody;
                Body body = bodies[nextBody];

                // Clear visitation mark
                body.IsMarked = false;

                nextBody = body.IslandNext;
            }

            Debug.Assert(index == bodyCount);

            // Clear contact island flags. Only need to consider contacts
            // already in the base island.
            int nextContactId = baseIsland.HeadContact;
            while (nextContactId != Core.NullIndex)
            {
                Contact contact = contacts[nextContactId];
                contact.IsMarked = false;
                nextContactId = contact.IslandNext;
            }

            // Clear joint island flags.
            int nextJoint = baseIsland.HeadJoint;
            while (nextJoint != Core.NullIndex)
            {
                Joint joint = Joint.GetJoint(world, nextJoint);
                joint.IsMarked = false;
                nextJoint = joint.IslandNext;
            }

            // Done with the base split island.
            DestroyIsland(world, baseId);

            // Each island is found as a depth first search starting from a seed body
            for (int i = 0; i < bodyCount; ++i)
            {
                int seedIndex = bodyIds[i];
                Body seed = bodies[seedIndex];
                Debug.Assert(seed.SetIndex == setIndex);

                if (seed.IsMarked == true)
                {
                    // The body has already been visited
                    continue;
                }

                int stackCount = 0;
                stack[stackCount++] = seedIndex;
                seed.IsMarked = true;

                // Create new island
                // No lock needed because only a single island can split per time step. No islands are being used during the constraint
                // solve. However, islands are touched during body finalization.
                Island island = CreateIsland(world, setIndex);

                int islandId = island.IslandId;

                // Perform a depth first search (DFS) on the constraint graph.
                while (stackCount > 0)
                {
                    // Grab the next body off the stack and add it to the island.
                    int bodyId = stack[--stackCount];
                    Body body = bodies[bodyId];
                    Debug.Assert(body.SetIndex == SolverSetType.AwakeSet);
                    Debug.Assert(body.IsMarked == true);

                    // Add body to island
                    body.IslandId = islandId;
                    if (island.TailBody != Core.NullIndex)
                    {
                        bodies[island.TailBody].IslandNext = bodyId;
                    }

                    body.IslandPrev = island.TailBody;
                    body.IslandNext = Core.NullIndex;
                    island.TailBody = bodyId;

                    if (island.HeadBody == Core.NullIndex)
                    {
                        island.HeadBody = bodyId;
                    }

                    island.BodyCount += 1;

                    // Search all contacts connected to this body.
                    int contactKey = body.HeadContactKey;
                    while (contactKey != Core.NullIndex)
                    {
                        int contactId = contactKey >> 1;
                        int edgeIndex = contactKey & 1;

                        world.ContactArray.CheckIndex(contactId);
                        Contact contact = world.ContactArray[contactId];
                        Debug.Assert(contact.ContactId == contactId);

                        // Next key
                        contactKey = contact.Edges[edgeIndex].NextKey;

                        // Has this contact already been added to this island?
                        if (contact.IsMarked)
                        {
                            continue;
                        }

                        // Skip sensors
                        if (contact.Flags.IsSet(ContactFlags.ContactSensorFlag))
                        {
                            continue;
                        }

                        // Is this contact enabled and touching?
                        if (contact.Flags.IsSet(ContactFlags.ContactTouchingFlag) == false)
                        {
                            continue;
                        }

                        contact.IsMarked = true;

                        int otherEdgeIndex = edgeIndex ^ 1;
                        int otherBodyId = contact.Edges[otherEdgeIndex].BodyId;
                        Body otherBody = bodies[otherBodyId];

                        // Maybe add other body to stack
                        if (otherBody.IsMarked == false && otherBody.SetIndex != SolverSetType.StaticSet)
                        {
                            Debug.Assert(stackCount < bodyCount);
                            stack[stackCount++] = otherBodyId;
                            otherBody.IsMarked = true;
                        }

                        // Add contact to island
                        contact.IslandId = islandId;
                        if (island.TailContact != Core.NullIndex)
                        {
                            world.ContactArray.CheckIndex(island.TailContact);
                            Contact tailContact = world.ContactArray[island.TailContact];
                            tailContact.IslandNext = contactId;
                        }

                        contact.IslandPrev = island.TailContact;
                        contact.IslandNext = Core.NullIndex;
                        island.TailContact = contactId;

                        if (island.HeadContact == Core.NullIndex)
                        {
                            island.HeadContact = contactId;
                        }

                        island.ContactCount += 1;
                    }

                    // Search all joints connect to this body.
                    int jointKey = body.HeadJointKey;
                    while (jointKey != Core.NullIndex)
                    {
                        int jointId = jointKey >> 1;
                        int edgeIndex = jointKey & 1;

                        Joint joint = Joint.GetJoint(world, jointId);
                        Debug.Assert(joint.JointId == jointId);

                        // Next key
                        jointKey = joint.Edges[edgeIndex].NextKey;

                        // Has this joint already been added to this island?
                        if (joint.IsMarked)
                        {
                            continue;
                        }

                        joint.IsMarked = true;

                        int otherEdgeIndex = edgeIndex ^ 1;
                        int otherBodyId = joint.Edges[otherEdgeIndex].BodyId;
                        Body otherBody = bodies[otherBodyId];

                        // Don't simulate joints connected to disabled bodies.
                        if (otherBody.SetIndex == SolverSetType.DisabledSet)
                        {
                            continue;
                        }

                        // Maybe add other body to stack
                        if (otherBody.IsMarked == false && otherBody.SetIndex == SolverSetType.AwakeSet)
                        {
                            Debug.Assert(stackCount < bodyCount);
                            stack[stackCount++] = otherBodyId;
                            otherBody.IsMarked = true;
                        }

                        // Add joint to island
                        joint.IslandId = islandId;
                        if (island.TailJoint != Core.NullIndex)
                        {
                            Joint tailJoint = Joint.GetJoint(world, island.TailJoint);
                            tailJoint.IslandNext = jointId;
                        }

                        joint.IslandPrev = island.TailJoint;
                        joint.IslandNext = Core.NullIndex;
                        island.TailJoint = jointId;

                        if (island.HeadJoint == Core.NullIndex)
                        {
                            island.HeadJoint = jointId;
                        }

                        island.JointCount += 1;
                    }
                }

                ValidateIsland(world, islandId);
            }

            B2ArrayPool<int>.Shared.Return(bodyIdsArray, true);
            B2ArrayPool<int>.Shared.Return(stackArray, true);
        }

        // Split an island because some contacts and/or joints have been removed.
        // This is called during the constraint solve while islands are not being touched. This uses DFS and touches a lot of memory,
        // so it can be quite slow.
        // Note: contacts/joints connected to static bodies must belong to an island but don't affect island connectivity
        // Note: static bodies are never in an island
        // Note: this task interacts with some allocators without locks under the assumption that no other tasks
        // are interacting with these data structures.
        public static void SplitIslandTask(World world)
        {
            var start = Stopwatch.GetTimestamp();

            Debug.Assert(world.SplitIslandId != Core.NullIndex);

            SplitIsland(world, world.SplitIslandId);
            var end = Stopwatch.GetTimestamp();
            world.Profile.SplitIslands += (float)StopwatchHelper.GetElapsedTime(start, end).TotalMilliseconds;
        }
    }

    public class IslandSim
    {
        public int IslandId;

        public void CopyTo(IslandSim islandDst)
        {
            islandDst.IslandId = IslandId;
        }

        public void Reset()
        {
            IslandId = 0;
        }
    }
}