using System;
using System.Diagnostics;

namespace Box2DSharp
{
    /// <summary>
    /// Cold contact data. Used as a persistent handle and for persistent island connectivity.
    /// </summary>
    public class Contact
    {
        /// <summary>
        /// index of simulation set stored in b2World
        /// B2_NULL_INDEX when slot is free
        /// 在World中模拟集合的索引
        /// </summary>
        public int SetIndex = Core.NullIndex;

        /// <summary>
        /// index into the constraint graph color array
        /// B2_NULL_INDEX for non-touching or sleeping contacts
        /// B2_NULL_INDEX when slot is free
        /// 在着色图数组中的索引，接触点是空闲对象/非触碰/休眠时，该值为<see cref="Core.NullIndex"/>
        /// </summary>
        public int ColorIndex = Core.NullIndex;

        /// <summary>
        /// contact index within set or graph color
        /// B2_NULL_INDEX when slot is free
        /// 在接触点集合或着色图中的本地索引，接触点是空闲对象时，该值为<see cref="Core.NullIndex"/>
        /// </summary>
        public int LocalIndex = Core.NullIndex;

        public ContactEdge[] Edges = new ContactEdge[2];

        public int ShapeIdA;

        public int ShapeIdB;

        /// <summary>
        /// A contact only belongs to an island if touching, otherwise B2_NULL_INDEX.
        /// </summary>
        public int IslandPrev;

        public int IslandNext;

        public int IslandId;

        public int ContactId = Core.NullIndex;

        public ContactFlags Flags;

        public bool IsMarked;

        public static float MixFriction(float friction1, float friction2)
        {
            return (float)Math.Sqrt(friction1 * friction2);
        }

        // Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
        // For example, a superball bounces on anything.
        public static float MixRestitution(float restitution1, float restitution2)
        {
            return restitution1 > restitution2 ? restitution1 : restitution2;
        }

        private static Manifold CircleManifold(
            in Shape shapeA,
            in Transform xfA,
            in Shape shapeB,
            in Transform xfB,
            ref DistanceCache cache)
        {
            return ManifoldFunc.CollideCircles(shapeA.Union.Circle, xfA, shapeB.Union.Circle, xfB);
        }

        private static Manifold CapsuleAndCircleManifold(
            in Shape shapeA,
            in Transform xfA,
            in Shape shapeB,
            in Transform xfB,
            ref DistanceCache cache)
        {
            return ManifoldFunc.CollideCapsuleAndCircle(shapeA.Union.Capsule, xfA, shapeB.Union.Circle, xfB);
        }

        private static Manifold CapsuleManifold(
            in Shape shapeA,
            in Transform xfA,
            in Shape shapeB,
            in Transform xfB,
            ref DistanceCache cache)
        {
            return ManifoldFunc.CollideCapsules(in shapeA.Union.Capsule, in xfA, in shapeB.Union.Capsule, xfB);
        }

        private static Manifold PolygonAndCircleManifold(
            in Shape shapeA,
            in Transform xfA,
            in Shape shapeB,
            in Transform xfB,
            ref DistanceCache cache)
        {
            return ManifoldFunc.CollidePolygonAndCircle(shapeA.Union.Polygon, xfA, shapeB.Union.Circle, xfB);
        }

        private static Manifold PolygonAndCapsuleManifold(
            in Shape shapeA,
            in Transform xfA,
            in Shape shapeB,
            in Transform xfB,
            ref DistanceCache cache)
        {
            return ManifoldFunc.CollidePolygonAndCapsule(shapeA.Union.Polygon, xfA, shapeB.Union.Capsule, xfB);
        }

        private static Manifold PolygonManifold(
            in Shape shapeA,
            in Transform xfA,
            in Shape shapeB,
            in Transform xfB,
            ref DistanceCache cache)
        {
            return ManifoldFunc.CollidePolygons(shapeA.Union.Polygon, xfA, shapeB.Union.Polygon, xfB);
        }

        private static Manifold SegmentAndCircleManifold(
            in Shape shapeA,
            in Transform xfA,
            in Shape shapeB,
            in Transform xfB,
            ref DistanceCache cache)
        {
            return ManifoldFunc.CollideSegmentAndCircle(shapeA.Union.Segment, xfA, shapeB.Union.Circle, xfB);
        }

        private static Manifold SegmentAndCapsuleManifold(
            in Shape shapeA,
            in Transform xfA,
            in Shape shapeB,
            in Transform xfB,
            ref DistanceCache cache)
        {
            return ManifoldFunc.CollideSegmentAndCapsule(shapeA.Union.Segment, xfA, shapeB.Union.Capsule, xfB);
        }

        private static Manifold SegmentAndPolygonManifold(
            in Shape shapeA,
            in Transform xfA,
            in Shape shapeB,
            in Transform xfB,
            ref DistanceCache cache)
        {
            return ManifoldFunc.CollideSegmentAndPolygon(shapeA.Union.Segment, xfA, shapeB.Union.Polygon, xfB);
        }

        private static Manifold ChainSegmentAndCircleManifold(
            in Shape shapeA,
            in Transform xfA,
            in Shape shapeB,
            in Transform xfB,
            ref DistanceCache cache)
        {
            return ManifoldFunc.CollideChainSegmentAndCircle(shapeA.Union.ChainSegment, xfA, shapeB.Union.Circle, xfB);
        }

        private static Manifold ChainSegmentAndCapsuleManifold(
            in Shape shapeA,
            in Transform xfA,
            in Shape shapeB,
            in Transform xfB,
            ref DistanceCache cache)
        {
            return ManifoldFunc.CollideChainSegmentAndCapsule(shapeA.Union.ChainSegment, xfA, shapeB.Union.Capsule, xfB, ref cache);
        }

        private static Manifold ChainSegmentAndPolygonManifold(
            in Shape shapeA,
            in Transform xfA,
            in Shape shapeB,
            in Transform xfB,
            ref DistanceCache cache)
        {
            return ManifoldFunc.CollideChainSegmentAndPolygon(shapeA.Union.ChainSegment, xfA, shapeB.Union.Polygon, xfB, ref cache);
        }

        private static void AddType(ManifoldFcn fcn, ShapeType type1, ShapeType type2)
        {
            Debug.Assert(0 <= type1 && type1 < ShapeType.ShapeTypeCount);
            Debug.Assert(0 <= type2 && type2 < ShapeType.ShapeTypeCount);
            ContactRegister.Registers[(int)type1, (int)type2] = new ContactRegister(fcn, true);

            if (type1 != type2)
            {
                ContactRegister.Registers[(int)type2, (int)type1] = new ContactRegister(fcn, false);
            }
        }

        public static void InitializeContactRegisters()
        {
            if (ContactRegister.Initialized == false)
            {
                AddType(CircleManifold, ShapeType.CircleShape, ShapeType.CircleShape);
                AddType(CapsuleAndCircleManifold, ShapeType.CapsuleShape, ShapeType.CircleShape);
                AddType(CapsuleManifold, ShapeType.CapsuleShape, ShapeType.CapsuleShape);
                AddType(PolygonAndCircleManifold, ShapeType.PolygonShape, ShapeType.CircleShape);
                AddType(PolygonAndCapsuleManifold, ShapeType.PolygonShape, ShapeType.CapsuleShape);
                AddType(PolygonManifold, ShapeType.PolygonShape, ShapeType.PolygonShape);
                AddType(SegmentAndCircleManifold, ShapeType.SegmentShape, ShapeType.CircleShape);
                AddType(SegmentAndCapsuleManifold, ShapeType.SegmentShape, ShapeType.CapsuleShape);
                AddType(SegmentAndPolygonManifold, ShapeType.SegmentShape, ShapeType.PolygonShape);
                AddType(ChainSegmentAndCircleManifold, ShapeType.ChainSegmentShape, ShapeType.CircleShape);
                AddType(ChainSegmentAndCapsuleManifold, ShapeType.ChainSegmentShape, ShapeType.CapsuleShape);
                AddType(ChainSegmentAndPolygonManifold, ShapeType.ChainSegmentShape, ShapeType.PolygonShape);
                ContactRegister.Initialized = true;
            }
        }

        public static void CreateContact(World world, ref Shape shapeA, ref Shape shapeB)
        {
            int type1 = (int)shapeA.Type;
            int type2 = (int)shapeB.Type;

            Debug.Assert(0 <= type1 && type1 < (int)ShapeType.ShapeTypeCount);
            Debug.Assert(0 <= type2 && type2 < (int)ShapeType.ShapeTypeCount);

            if (ContactRegister.Registers[type1, type2].Fcn == null!)
            {
                // For example, no segment vs segment collision
                return;
            }

            if (ContactRegister.Registers[type1, type2].Primary == false)
            {
                // flip order
                CreateContact(world, ref shapeB, ref shapeA);
                return;
            }

            Body bodyA = Body.GetBody(world, shapeA.BodyId);
            Body bodyB = Body.GetBody(world, shapeB.BodyId);

            Debug.Assert(bodyA.SetIndex != SolverSetType.DisabledSet && bodyB.SetIndex != SolverSetType.DisabledSet);
            Debug.Assert(bodyA.SetIndex != SolverSetType.StaticSet || bodyB.SetIndex != SolverSetType.StaticSet);

            int setIndex;
            if (bodyA.SetIndex == SolverSetType.AwakeSet || bodyB.SetIndex == SolverSetType.AwakeSet)
            {
                setIndex = SolverSetType.AwakeSet;
            }
            else
            {
                // sleeping and non-touching contacts live in the disabled set
                // later if this set is found to be touching then the sleeping
                // islands will be linked and the contact moved to the merged island
                setIndex = SolverSetType.DisabledSet;
            }

            ref SolverSet set = ref world.SolverSetArray[setIndex];

            // Create contact key and contact
            int contactId = world.ContactIdPool.AllocId();
            if (contactId == world.ContactArray.Count)
            {
                world.ContactArray.Push(new Contact());
            }

            int shapeIdA = shapeA.Id;
            int shapeIdB = shapeB.Id;

            ref Contact contact = ref world.ContactArray[contactId];
            contact.ContactId = contactId;
            contact.SetIndex = setIndex;
            contact.ColorIndex = Core.NullIndex;
            contact.LocalIndex = set.Contacts.Count;
            contact.IslandId = Core.NullIndex;
            contact.IslandPrev = Core.NullIndex;
            contact.IslandNext = Core.NullIndex;
            contact.ShapeIdA = shapeIdA;
            contact.ShapeIdB = shapeIdB;
            contact.IsMarked = false;
            contact.Flags = 0;

            if (shapeA.IsSensor || shapeB.IsSensor)
            {
                contact.Flags |= ContactFlags.ContactSensorFlag;
            }

            if (shapeA.EnableSensorEvents || shapeB.EnableSensorEvents)
            {
                contact.Flags |= ContactFlags.ContactEnableSensorEvents;
            }

            if (shapeA.EnableContactEvents || shapeB.EnableContactEvents)
            {
                contact.Flags |= ContactFlags.ContactEnableContactEvents;
            }

            // Connect to body A
            {
                contact.Edges[0].BodyId = shapeA.BodyId;
                contact.Edges[0].PrevKey = Core.NullIndex;
                contact.Edges[0].NextKey = bodyA.HeadContactKey;

                int keyA = (contactId << 1) | 0;
                int headContactKey = bodyA.HeadContactKey;
                if (headContactKey != Core.NullIndex)
                {
                    ref Contact headContact = ref world.ContactArray[headContactKey >> 1];
                    headContact.Edges[headContactKey & 1].PrevKey = keyA;
                }

                bodyA.HeadContactKey = keyA;
                bodyA.ContactCount += 1;
            }

            // Connect to body B
            {
                contact.Edges[1].BodyId = shapeB.BodyId;
                contact.Edges[1].PrevKey = Core.NullIndex;
                contact.Edges[1].NextKey = bodyB.HeadContactKey;

                int keyB = (contactId << 1) | 1;
                int headContactKey = bodyB.HeadContactKey;
                if (bodyB.HeadContactKey != Core.NullIndex)
                {
                    ref Contact headContact = ref world.ContactArray[headContactKey >> 1];
                    headContact.Edges[headContactKey & 1].PrevKey = keyB;
                }

                bodyB.HeadContactKey = keyB;
                bodyB.ContactCount += 1;
            }

            // Add to pair set for fast lookup
            var pairKey = Core.ShapePairKey(shapeIdA, shapeIdB);
            world.BroadPhase.PairSet.AddKey(pairKey);

            // Contacts are created as non-touching. Later if they are found to be touching
            // they will link islands and be moved into the constraint graph.
            ContactSim contactSim = set.Contacts.AddContact();
            contactSim.ContactId = contactId;

            //#if B2_VALIDATE
            contactSim.BodyIdA = shapeA.BodyId;
            contactSim.BodyIdB = shapeB.BodyId;

            //#endif

            contactSim.BodySimIndexA = Core.NullIndex;
            contactSim.BodySimIndexB = Core.NullIndex;
            contactSim.InvMassA = 0.0f;
            contactSim.InvIA = 0.0f;
            contactSim.InvMassB = 0.0f;
            contactSim.InvIB = 0.0f;
            contactSim.ShapeIdA = shapeIdA;
            contactSim.ShapeIdB = shapeIdB;
            contactSim.Cache = new DistanceCache();
            contactSim.Manifold = new Manifold();
            contactSim.Friction = MixFriction(shapeA.Friction, shapeB.Friction);
            contactSim.Restitution = MixRestitution(shapeA.Restitution, shapeB.Restitution);
            contactSim.TangentSpeed = 0.0f;
            contactSim.SimFlags = 0;

            if (shapeA.EnablePreSolveEvents || shapeB.EnablePreSolveEvents)
            {
                contactSim.SimFlags |= ContactSimFlags.EnablePreSolveEvents;
            }
        }

        // A contact is destroyed when:
        // - broad-phase proxies stop overlapping
        // - a body is destroyed
        // - a body is disabled
        // - a body changes type from dynamic to kinematic or static
        // - a shape is destroyed
        // - contact filtering is modified
        // - a shape becomes a sensor (check this!!!)
        public static void DestroyContact(World world, Contact contact, bool wakeBodies)
        {
            // Remove pair from set
            var pairKey = Core.ShapePairKey(contact.ShapeIdA, contact.ShapeIdB);
            world.BroadPhase.PairSet.RemoveKey(pairKey);

            ref ContactEdge edgeA = ref contact.Edges[0];
            ref ContactEdge edgeB = ref contact.Edges[1];

            int bodyIdA = edgeA.BodyId;
            int bodyIdB = edgeB.BodyId;
            Body bodyA = Body.GetBody(world, bodyIdA);
            Body bodyB = Body.GetBody(world, bodyIdB);

            // if (contactListener && contact.IsTouching())
            //{
            //	contactListener.EndContact(contact);
            // }

            // Remove from body A
            if (edgeA.PrevKey != Core.NullIndex)
            {
                ref readonly Contact prevContact = ref world.ContactArray[(edgeA.PrevKey >> 1)];
                ref ContactEdge prevEdge = ref prevContact.Edges[(edgeA.PrevKey & 1)];
                prevEdge.NextKey = edgeA.NextKey;
            }

            if (edgeA.NextKey != Core.NullIndex)
            {
                ref readonly Contact nextContact = ref world.ContactArray[(edgeA.NextKey >> 1)];
                ref ContactEdge nextEdge = ref nextContact.Edges[(edgeA.NextKey & 1)];
                nextEdge.PrevKey = edgeA.PrevKey;
            }

            int contactId = contact.ContactId;

            int edgeKeyA = (contactId << 1) | 0;
            if (bodyA.HeadContactKey == edgeKeyA)
            {
                bodyA.HeadContactKey = edgeA.NextKey;
            }

            bodyA.ContactCount -= 1;

            // Remove from body B
            if (edgeB.PrevKey != Core.NullIndex)
            {
                ref readonly Contact prevContact = ref world.ContactArray[(edgeB.PrevKey >> 1)];
                ref ContactEdge prevEdge = ref prevContact.Edges[(edgeB.PrevKey & 1)];
                prevEdge.NextKey = edgeB.NextKey;
            }

            if (edgeB.NextKey != Core.NullIndex)
            {
                ref readonly Contact nextContact = ref world.ContactArray[(edgeB.NextKey >> 1)];
                ref ContactEdge nextEdge = ref nextContact.Edges[(edgeB.NextKey & 1)];
                nextEdge.PrevKey = edgeB.PrevKey;
            }

            int edgeKeyB = (contactId << 1) | 1;
            if (bodyB.HeadContactKey == edgeKeyB)
            {
                bodyB.HeadContactKey = edgeB.NextKey;
            }

            bodyB.ContactCount -= 1;

            // Remove contact from the array that owns it
            if (contact.IslandId != Core.NullIndex)
            {
                Island.UnlinkContact(world, contact);
            }

            if (contact.ColorIndex != Core.NullIndex)
            {
                // contact is an active constraint
                Debug.Assert(contact.SetIndex == SolverSetType.AwakeSet);
                ConstraintGraph.RemoveContactFromGraph(world, bodyIdA, bodyIdB, contact.ColorIndex, contact.LocalIndex);
            }
            else
            {
                // contact is non-touching or is sleeping or is a sensor
                Debug.Assert(contact.SetIndex != SolverSetType.AwakeSet || (contact.Flags & ContactFlags.ContactTouchingFlag) == 0 || (contact.Flags & ContactFlags.ContactSensorFlag) != 0);
                var set = world.SolverSetArray[contact.SetIndex];
                var movedIndex = set.Contacts.RemoveContact(contact.LocalIndex);
                if (movedIndex != Core.NullIndex)
                {
                    var movedContact = set.Contacts.Data[contact.LocalIndex];
                    world.ContactArray[movedContact.ContactId].LocalIndex = contact.LocalIndex;
                }
            }

            contact.ContactId = Core.NullIndex;
            contact.SetIndex = Core.NullIndex;
            contact.ColorIndex = Core.NullIndex;
            contact.LocalIndex = Core.NullIndex;

            world.ContactIdPool.FreeId(contactId);

            if (wakeBodies)
            {
                Body.WakeBody(world, bodyA);
                Body.WakeBody(world, bodyB);
            }
        }

        public static ContactSim GetContactSim(World world, Contact contact)
        {
            if (contact.SetIndex == SolverSetType.AwakeSet && contact.ColorIndex != Core.NullIndex)
            {
                // contact lives in constraint graph
                Debug.Assert(0 <= contact.ColorIndex && contact.ColorIndex < Core.GraphColorCount);
                ref GraphColor color = ref world.ConstraintGraph.Colors[contact.ColorIndex];
                Debug.Assert(0 <= contact.LocalIndex && contact.LocalIndex < color.Contacts.Count);
                return color.Contacts.Data[contact.LocalIndex];
            }

            ref SolverSet set = ref world.SolverSetArray[contact.SetIndex];
            Debug.Assert(0 <= contact.LocalIndex && contact.LocalIndex <= set.Contacts.Count);
            return set.Contacts.Data[contact.LocalIndex];
        }

        public static bool ShouldShapesCollide(Filter filterA, Filter filterB)
        {
            if (filterA.GroupIndex == filterB.GroupIndex && filterA.GroupIndex != 0)
            {
                return filterA.GroupIndex > 0;
            }

            bool collide = (filterA.MaskBits & filterB.CategoryBits) != 0 && (filterA.CategoryBits & filterB.MaskBits) != 0;
            return collide;
        }

        public static bool TestShapeOverlap(
            ref Shape shapeA,
            Transform xfA,
            ref Shape shapeB,
            Transform xfB,
            ref DistanceCache cache)
        {
            DistanceInput input;
            input.ProxyA = shapeA.MakeShapeDistanceProxy();
            input.ProxyB = shapeB.MakeShapeDistanceProxy();
            input.TransformA = xfA;
            input.TransformB = xfB;
            input.UseRadii = true;

            DistanceOutput output = DistanceFunc.ShapeDistance(ref cache, input, null, 0);

            return output.Distance < 10.0f * float.Epsilon;
        }

        // Update the contact manifold and touching status. Also updates sensor overlap.
        // Note: do not assume the shape AABBs are overlapping or are valid.
        public static bool UpdateContact(
            World world,
            ContactSim contactSim,
            Shape shapeA,
            in Transform transformA,
            Vec2 centerOffsetA,
            Shape shapeB,
            in Transform transformB,
            Vec2 centerOffsetB)
        {
            bool touching;

            // Is this contact a sensor?
            if (shapeA.IsSensor || shapeB.IsSensor)
            {
                // Sensors don't generate manifolds or hit events
                touching = TestShapeOverlap(ref shapeA, transformA, ref shapeB, transformB, ref contactSim.Cache);
            }
            else
            {
                Manifold oldManifold = contactSim.Manifold;

                // Compute TOI
                ManifoldFcn fcn = ContactRegister.Registers[(int)shapeA.Type, (int)shapeB.Type].Fcn;

                contactSim.Manifold = fcn(shapeA, transformA, shapeB, transformB, ref contactSim.Cache);

                int pointCount = contactSim.Manifold.PointCount;
                touching = pointCount > 0;

                if (touching && world.PreSolveFcn != null && (contactSim.SimFlags & ContactSimFlags.EnablePreSolveEvents) != 0)
                {
                    ShapeId shapeIdA = new(shapeA.Id + 1, world.WorldId, shapeA.Revision);
                    ShapeId shapeIdB = new(shapeB.Id + 1, world.WorldId, shapeB.Revision);

                    // this call assumes thread safety
                    touching = world.PreSolveFcn(shapeIdA, shapeIdB, ref contactSim.Manifold, world.PreSolveContext);
                    if (touching == false)
                    {
                        // disable contact
                        contactSim.Manifold.PointCount = 0;
                    }
                }

                if (touching && (shapeA.EnableHitEvents || shapeB.EnableHitEvents))
                {
                    contactSim.SimFlags |= ContactSimFlags.EnableHitEvent;
                }
                else
                {
                    contactSim.SimFlags &= ~ContactSimFlags.EnableHitEvent;
                }

                // Match old contact ids to new contact ids and copy the
                // stored impulses to warm start the solver.
                for (int i = 0; i < pointCount; ++i)
                {
                    ref ManifoldPoint mp2 = ref contactSim.Manifold.Points[i];

                    // shift anchors to be center of mass relative
                    mp2.AnchorA = B2Math.Sub(mp2.AnchorA, centerOffsetA);
                    mp2.AnchorB = B2Math.Sub(mp2.AnchorB, centerOffsetB);

                    mp2.NormalImpulse = 0.0f;
                    mp2.TangentImpulse = 0.0f;
                    mp2.MaxNormalImpulse = 0.0f;
                    mp2.NormalVelocity = 0.0f;
                    mp2.Persisted = false;

                    var id2 = mp2.Id;

                    for (int j = 0; j < oldManifold.PointCount; ++j)
                    {
                        ref ManifoldPoint mp1 = ref oldManifold.Points[j];

                        if (mp1.Id == id2)
                        {
                            mp2.NormalImpulse = mp1.NormalImpulse;
                            mp2.TangentImpulse = mp1.TangentImpulse;
                            mp2.Persisted = true;
                            break;
                        }
                    }
                }
            }

            if (touching)
            {
                contactSim.SimFlags |= ContactSimFlags.TouchingFlag;
            }
            else
            {
                contactSim.SimFlags &= ~ContactSimFlags.TouchingFlag;
            }

            return touching;
        }
    }

    // todo make relative for all
    // typedef b2Manifold b2ManifoldFcn(in b2Shape shapeA, in b2Shape shapeB, b2Transform xfB, ref b2DistanceCache cache);
}