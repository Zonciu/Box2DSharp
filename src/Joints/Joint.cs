using System;
using System.Diagnostics;

namespace Box2DSharp
{
    // Map from b2JointId to b2Joint in the solver sets
    public class Joint
    {
        public object? UserData;

        // index of simulation set stored in b2World
        // B2_NULL_INDEX when slot is free
        public int SetIndex;

        // index into the constraint graph color array, may be B2_NULL_INDEX for sleeping/disabled joints
        // B2_NULL_INDEX when slot is free
        public int ColorIndex;

        // joint index within set or graph color
        // B2_NULL_INDEX when slot is free
        public int LocalIndex;

        public JointEdge[] Edges = new JointEdge[2];

        public int JointId;

        public int IslandId;

        public int IslandPrev;

        public int IslandNext;

        // This is monotonically advanced when a body is allocated in this slot
        // Used to check for invalid b2JointId
        public int Revision;

        public float DrawSize;

        public JointType Type;

        public bool IsMarked;

        public bool CollideConnected;

        public static MotorJointDef DefaultMotorJointDef()
        {
            MotorJointDef def = new();
            def.MaxForce = 1.0f;
            def.MaxTorque = 1.0f;
            def.CorrectionFactor = 0.3f;
            def.InternalValue = Core.SecretCookie;
            return def;
        }

        public static PrismaticJointDef DefaultPrismaticJointDef()
        {
            PrismaticJointDef def = new();
            def.LocalAxisA = new Vec2(1.0f, 0.0f);
            def.InternalValue = Core.SecretCookie;
            return def;
        }

        public static WeldJointDef DefaultWeldJointDef()
        {
            WeldJointDef def = new();
            def.InternalValue = Core.SecretCookie;
            return def;
        }

        public static Joint GetJointFullId(World world, JointId jointId)
        {
            int id = jointId.Index1 - 1;
            world.JointArray.CheckIndex(id);
            Joint joint = world.JointArray[id];
            world.SolverSetArray.CheckIndex(joint.SetIndex);
            Debug.Assert(joint.Revision == jointId.Revision);
            return joint;
        }

        public static Joint GetJoint(World world, int jointId)
        {
            world.JointArray.CheckIndex(jointId);
            Joint joint = world.JointArray[jointId];
            return joint;
        }

        public static JointSim GetJointSim(World world, Joint joint)
        {
            world.SolverSetArray.CheckIndex(joint.SetIndex);

            if (joint.SetIndex == SolverSetType.AwakeSet)
            {
                Debug.Assert(0 <= joint.ColorIndex && joint.ColorIndex < Core.GraphColorCount);
                GraphColor color = world.ConstraintGraph.Colors[joint.ColorIndex];
                Debug.Assert(0 <= joint.LocalIndex && joint.LocalIndex < color.Joints.Count);
                return color.Joints.Data[joint.LocalIndex];
            }

            SolverSet set = world.SolverSetArray[joint.SetIndex];
            Debug.Assert(0 <= joint.LocalIndex && joint.LocalIndex < set.Joints.Count);
            return set.Joints.Data[joint.LocalIndex];
        }

        public static JointSim GetJointSimCheckType(JointId jointId, JointType type)
        {
            World world = World.GetWorld(jointId.World0);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return null;
            }

            Joint joint = GetJointFullId(world, jointId);
            Debug.Assert(joint.Type == type);
            JointSim jointSim = GetJointSim(world, joint);
            Debug.Assert(jointSim.Type == type);
            return jointSim;
        }

        public class JointPair
        {
            public Joint Joint;

            public JointSim JointSim;

            public JointPair(Joint joint, JointSim jointSim)
            {
                Joint = joint;
                JointSim = jointSim;
            }
        }

        public static JointPair CreateJoint(
            World world,
            Body bodyA,
            Body bodyB,
            object userData,
            float drawSize,
            JointType type,
            bool collideConnected)
        {
            int bodyIdA = bodyA.Id;
            int bodyIdB = bodyB.Id;
            int maxSetIndex = Math.Max(bodyA.SetIndex, bodyB.SetIndex);

            // Create joint id and joint
            int jointId = world.JointIdPool.AllocId();
            if (jointId == world.JointArray.Count)
            {
                world.JointArray.Push(new Joint());
            }

            Joint joint = world.JointArray[jointId];
            joint.JointId = jointId;
            joint.UserData = userData;
            joint.Revision += 1;
            joint.SetIndex = Core.NullIndex;
            joint.ColorIndex = Core.NullIndex;
            joint.LocalIndex = Core.NullIndex;
            joint.IslandId = Core.NullIndex;
            joint.IslandPrev = Core.NullIndex;
            joint.IslandNext = Core.NullIndex;
            joint.DrawSize = drawSize;
            joint.Type = type;
            joint.CollideConnected = collideConnected;
            joint.IsMarked = false;

            // Doubly linked list on bodyA
            joint.Edges[0].BodyId = bodyIdA;
            joint.Edges[0].PrevKey = Core.NullIndex;
            joint.Edges[0].NextKey = bodyA.HeadJointKey;

            int keyA = (jointId << 1) | 0;
            if (bodyA.HeadJointKey != Core.NullIndex)
            {
                Joint jointA = world.JointArray[bodyA.HeadJointKey >> 1];
                ref JointEdge edgeA = ref jointA.Edges[bodyA.HeadJointKey & 1];
                edgeA.PrevKey = keyA;
            }

            bodyA.HeadJointKey = keyA;
            bodyA.JointCount += 1;

            // Doubly linked list on bodyB
            joint.Edges[1].BodyId = bodyIdB;
            joint.Edges[1].PrevKey = Core.NullIndex;
            joint.Edges[1].NextKey = bodyB.HeadJointKey;

            int keyB = (jointId << 1) | 1;
            if (bodyB.HeadJointKey != Core.NullIndex)
            {
                Joint jointB = world.JointArray[bodyB.HeadJointKey >> 1];
                ref JointEdge edgeB = ref jointB.Edges[bodyB.HeadJointKey & 1];
                edgeB.PrevKey = keyB;
            }

            bodyB.HeadJointKey = keyB;
            bodyB.JointCount += 1;

            JointSim jointSim;

            if (bodyA.SetIndex == SolverSetType.DisabledSet || bodyB.SetIndex == SolverSetType.DisabledSet)
            {
                // if either body is disabled, create in disabled set
                SolverSet set = world.SolverSetArray[SolverSetType.DisabledSet];
                joint.SetIndex = SolverSetType.DisabledSet;
                joint.LocalIndex = set.Joints.Count;

                jointSim = set.Joints.AddJoint();
                jointSim.JointId = jointId;
                jointSim.BodyIdA = bodyIdA;
                jointSim.BodyIdB = bodyIdB;
            }
            else if (bodyA.SetIndex == SolverSetType.StaticSet && bodyB.SetIndex == SolverSetType.StaticSet)
            {
                // joint is connecting static bodies
                SolverSet set = world.SolverSetArray[SolverSetType.StaticSet];
                joint.SetIndex = SolverSetType.StaticSet;
                joint.LocalIndex = set.Joints.Count;

                jointSim = set.Joints.AddJoint();
                jointSim.JointId = jointId;
                jointSim.BodyIdA = bodyIdA;
                jointSim.BodyIdB = bodyIdB;
            }
            else if (bodyA.SetIndex == SolverSetType.AwakeSet || bodyB.SetIndex == SolverSetType.AwakeSet)
            {
                // if either body is sleeping, wake it
                if (maxSetIndex >= SolverSetType.FirstSleepingSet)
                {
                    SolverSet.WakeSolverSet(world, maxSetIndex);
                }

                joint.SetIndex = SolverSetType.AwakeSet;

                jointSim = ConstraintGraph.CreateJointInGraph(world, joint);
                jointSim.JointId = jointId;
                jointSim.BodyIdA = bodyIdA;
                jointSim.BodyIdB = bodyIdB;
            }
            else
            {
                // joint connected between sleeping and/or static bodies
                Debug.Assert(bodyA.SetIndex >= SolverSetType.FirstSleepingSet || bodyB.SetIndex >= SolverSetType.FirstSleepingSet);
                Debug.Assert(bodyA.SetIndex != SolverSetType.StaticSet || bodyB.SetIndex != SolverSetType.StaticSet);

                // joint should go into the sleeping set (not static set)
                int setIndex = maxSetIndex;

                world.SolverSetArray.CheckIndex(setIndex);
                SolverSet set = world.SolverSetArray[setIndex];
                joint.SetIndex = setIndex;
                joint.LocalIndex = set.Joints.Count;
                jointSim = set.Joints.AddJoint();
                jointSim.JointId = jointId;
                jointSim.BodyIdA = bodyIdA;
                jointSim.BodyIdB = bodyIdB;

                if (bodyA.SetIndex != bodyB.SetIndex && bodyA.SetIndex >= SolverSetType.FirstSleepingSet && bodyB.SetIndex >= SolverSetType.FirstSleepingSet)
                {
                    // merge sleeping sets
                    SolverSet.MergeSolverSets(world, bodyA.SetIndex, bodyB.SetIndex);
                    Debug.Assert(bodyA.SetIndex == bodyB.SetIndex);

                    // fix potentially invalid set index
                    setIndex = bodyA.SetIndex;

                    // Careful! The joint sim pointer was orphaned by the set merge.
                    jointSim = world.SolverSetArray[setIndex].Joints.Data[joint.LocalIndex];
                }

                Debug.Assert(joint.SetIndex == setIndex);
            }

            Debug.Assert(jointSim.JointId == jointId);
            Debug.Assert(jointSim.BodyIdA == bodyIdA);
            Debug.Assert(jointSim.BodyIdB == bodyIdB);

            if (joint.SetIndex > SolverSetType.DisabledSet)
            {
                // Add edge to island graph
                bool mergeIslands = true;
                Island.LinkJoint(world, joint, mergeIslands);
            }

            SolverSet.ValidateSolverSets(world);

            return new JointPair(joint, jointSim);
        }

        public static void DestroyContactsBetweenBodies(World world, Body bodyA, Body bodyB)
        {
            int contactKey;
            int otherBodyId;

            // use the smaller of the two contact lists
            if (bodyA.ContactCount < bodyB.ContactCount)
            {
                contactKey = bodyA.HeadContactKey;
                otherBodyId = bodyB.Id;
            }
            else
            {
                contactKey = bodyB.HeadContactKey;
                otherBodyId = bodyA.Id;
            }

            // no need to wake bodies when a joint removes collision between them
            bool wakeBodies = false;

            // destroy the contacts
            while (contactKey != Core.NullIndex)
            {
                int contactId = contactKey >> 1;
                int edgeIndex = contactKey & 1;

                world.ContactArray.CheckIndex(contactId);
                ref var contact = ref world.ContactArray[contactId];
                contactKey = contact.Edges[edgeIndex].NextKey;

                int otherEdgeIndex = edgeIndex ^ 1;
                if (contact.Edges[otherEdgeIndex].BodyId == otherBodyId)
                {
                    // Careful, this removes the contact from the current doubly linked list
                    Contact.DestroyContact(world, contact, wakeBodies);
                }
            }

            SolverSet.ValidateSolverSets(world);
        }

        public static JointId CreateDistanceJoint(WorldId worldId, in DistanceJointDef def)

        {
            def.CheckDef();
            World world = World.GetWorldFromId(worldId);

            Debug.Assert(world.Locked == false);

            if (world.Locked)
            {
                return new JointId();
            }

            Debug.Assert(def.BodyIdA.IsValid());
            Debug.Assert(def.BodyIdB.IsValid());
            Debug.Assert(B2Math.IsValid(def.Length) && def.Length > 0.0f);

            Body bodyA = Body.GetBodyFullId(world, def.BodyIdA);
            Body bodyB = Body.GetBodyFullId(world, def.BodyIdB);

            JointPair pair = CreateJoint(world, bodyA, bodyB, def.UserData, 1.0f, JointType.DistanceJoint, def.CollideConnected);

            JointSim joint = pair.JointSim;
            joint.Type = JointType.DistanceJoint;
            joint.LocalOriginAnchorA = def.LocalAnchorA;
            joint.LocalOriginAnchorB = def.LocalAnchorB;

            DistanceJoint empty = new();
            joint.Joint.DistanceJoint = empty;
            joint.Joint.DistanceJoint.Length = Math.Max(def.Length, Core.LinearSlop);
            joint.Joint.DistanceJoint.Hertz = def.Hertz;
            joint.Joint.DistanceJoint.DampingRatio = def.DampingRatio;
            joint.Joint.DistanceJoint.MinLength = Math.Max(def.MinLength, Core.LinearSlop);
            joint.Joint.DistanceJoint.MaxLength = Math.Max(def.MinLength, def.MaxLength);
            joint.Joint.DistanceJoint.MaxMotorForce = def.MaxMotorForce;
            joint.Joint.DistanceJoint.MotorSpeed = def.MotorSpeed;
            joint.Joint.DistanceJoint.EnableSpring = def.EnableSpring;
            joint.Joint.DistanceJoint.EnableLimit = def.EnableLimit;
            joint.Joint.DistanceJoint.EnableMotor = def.EnableMotor;
            joint.Joint.DistanceJoint.Impulse = 0.0f;
            joint.Joint.DistanceJoint.LowerImpulse = 0.0f;
            joint.Joint.DistanceJoint.UpperImpulse = 0.0f;
            joint.Joint.DistanceJoint.MotorImpulse = 0.0f;

            // If the joint prevents collisions, then destroy all contacts between attached bodies
            if (def.CollideConnected == false)
            {
                DestroyContactsBetweenBodies(world, bodyA, bodyB);
            }

            JointId jointId = new(joint.JointId + 1, world.WorldId, (ushort)pair.Joint.Revision);
            return jointId;
        }

        public static JointId CreateMotorJoint(WorldId worldId, MotorJointDef def)

        {
            def.CheckDef();
            World world = World.GetWorldFromId(worldId);

            Debug.Assert(world.Locked == false);

            if (world.Locked)
            {
                return new JointId();
            }

            Body bodyA = Body.GetBodyFullId(world, def.BodyIdA);
            Body bodyB = Body.GetBodyFullId(world, def.BodyIdB);

            JointPair pair = CreateJoint(world, bodyA, bodyB, def.UserData, 1.0f, JointType.MotorJoint, def.CollideConnected);
            JointSim joint = pair.JointSim;

            joint.Type = JointType.MotorJoint;
            joint.LocalOriginAnchorA = Vec2.Zero;
            joint.LocalOriginAnchorB = Vec2.Zero;
            joint.Joint.MotorJoint = new MotorJoint();
            joint.Joint.MotorJoint.LinearOffset = def.LinearOffset;
            joint.Joint.MotorJoint.AngularOffset = def.AngularOffset;
            joint.Joint.MotorJoint.MaxForce = def.MaxForce;
            joint.Joint.MotorJoint.MaxTorque = def.MaxTorque;
            joint.Joint.MotorJoint.CorrectionFactor = Math.Clamp(def.CorrectionFactor, 0.0f, 1.0f);

            // If the joint prevents collisions, then destroy all contacts between attached bodies
            if (def.CollideConnected == false)
            {
                DestroyContactsBetweenBodies(world, bodyA, bodyB);
            }

            JointId jointId = new(joint.JointId + 1, world.WorldId, (ushort)pair.Joint.Revision);
            return jointId;
        }

        public static JointId CreateMouseJoint(WorldId worldId, in MouseJointDef def)

        {
            def.CheckDef();
            World world = World.GetWorldFromId(worldId);

            Debug.Assert(world.Locked == false);

            if (world.Locked)
            {
                return new JointId();
            }

            Body bodyA = Body.GetBodyFullId(world, def.BodyIdA);
            Body bodyB = Body.GetBodyFullId(world, def.BodyIdB);

            Transform transformA = Body.GetBodyTransformQuick(world, bodyA);
            Transform transformB = Body.GetBodyTransformQuick(world, bodyB);

            JointPair pair = CreateJoint(world, bodyA, bodyB, def.UserData, 1.0f, JointType.MouseJoint, def.CollideConnected);

            JointSim joint = pair.JointSim;
            joint.Type = JointType.MouseJoint;
            joint.LocalOriginAnchorA = B2Math.InvTransformPoint(transformA, def.Target);
            joint.LocalOriginAnchorB = B2Math.InvTransformPoint(transformB, def.Target);

            MouseJoint empty = new();
            joint.Joint.MouseJoint = empty;
            joint.Joint.MouseJoint.TargetA = def.Target;
            joint.Joint.MouseJoint.Hertz = def.Hertz;
            joint.Joint.MouseJoint.DampingRatio = def.DampingRatio;
            joint.Joint.MouseJoint.MaxForce = def.MaxForce;

            JointId jointId = new(joint.JointId + 1, world.WorldId, (ushort)pair.Joint.Revision);
            return jointId;
        }

        public static JointId CreateRevoluteJoint(WorldId worldId, in RevoluteJointDef def)

        {
            def.CheckDef();
            World world = World.GetWorldFromId(worldId);

            Debug.Assert(world.Locked == false);

            if (world.Locked)
            {
                return new JointId();
            }

            Body bodyA = Body.GetBodyFullId(world, def.BodyIdA);
            Body bodyB = Body.GetBodyFullId(world, def.BodyIdB);

            JointPair pair = CreateJoint(world, bodyA, bodyB, def.UserData, def.DrawSize, JointType.RevoluteJoint, def.CollideConnected);

            JointSim joint = pair.JointSim;
            joint.Type = JointType.RevoluteJoint;
            joint.LocalOriginAnchorA = def.LocalAnchorA;
            joint.LocalOriginAnchorB = def.LocalAnchorB;

            RevoluteJoint empty = new();
            joint.Joint.RevoluteJoint = empty;

            joint.Joint.RevoluteJoint.ReferenceAngle = Math.Clamp(def.ReferenceAngle, -B2Math.Pi, B2Math.Pi);
            joint.Joint.RevoluteJoint.LinearImpulse = Vec2.Zero;
            joint.Joint.RevoluteJoint.AxialMass = 0.0f;
            joint.Joint.RevoluteJoint.SpringImpulse = 0.0f;
            joint.Joint.RevoluteJoint.MotorImpulse = 0.0f;
            joint.Joint.RevoluteJoint.LowerImpulse = 0.0f;
            joint.Joint.RevoluteJoint.UpperImpulse = 0.0f;
            joint.Joint.RevoluteJoint.Hertz = def.Hertz;
            joint.Joint.RevoluteJoint.DampingRatio = def.DampingRatio;
            joint.Joint.RevoluteJoint.LowerAngle = Math.Min(def.LowerAngle, def.UpperAngle);
            joint.Joint.RevoluteJoint.UpperAngle = Math.Max(def.LowerAngle, def.UpperAngle);
            joint.Joint.RevoluteJoint.LowerAngle = Math.Clamp(joint.Joint.RevoluteJoint.LowerAngle, -B2Math.Pi, B2Math.Pi);
            joint.Joint.RevoluteJoint.UpperAngle = Math.Clamp(joint.Joint.RevoluteJoint.UpperAngle, -B2Math.Pi, B2Math.Pi);
            joint.Joint.RevoluteJoint.MaxMotorTorque = def.MaxMotorTorque;
            joint.Joint.RevoluteJoint.MotorSpeed = def.MotorSpeed;
            joint.Joint.RevoluteJoint.EnableSpring = def.EnableSpring;
            joint.Joint.RevoluteJoint.EnableLimit = def.EnableLimit;
            joint.Joint.RevoluteJoint.EnableMotor = def.EnableMotor;

            // If the joint prevents collisions, then destroy all contacts between attached bodies
            if (def.CollideConnected == false)
            {
                DestroyContactsBetweenBodies(world, bodyA, bodyB);
            }

            JointId jointId = new(joint.JointId + 1, world.WorldId, (ushort)pair.Joint.Revision);
            return jointId;
        }

        public static JointId CreatePrismaticJoint(WorldId worldId, in PrismaticJointDef def)

        {
            def.CheckDef();
            World world = World.GetWorldFromId(worldId);

            Debug.Assert(world.Locked == false);

            if (world.Locked)
            {
                return new JointId();
            }

            Body bodyA = Body.GetBodyFullId(world, def.BodyIdA);
            Body bodyB = Body.GetBodyFullId(world, def.BodyIdB);

            JointPair pair = CreateJoint(world, bodyA, bodyB, def.UserData, 1.0f, JointType.PrismaticJoint, def.CollideConnected);

            JointSim joint = pair.JointSim;
            joint.Type = JointType.PrismaticJoint;
            joint.LocalOriginAnchorA = def.LocalAnchorA;
            joint.LocalOriginAnchorB = def.LocalAnchorB;

            PrismaticJoint empty = new();
            joint.Joint.PrismaticJoint = empty;

            joint.Joint.PrismaticJoint.LocalAxisA = def.LocalAxisA.Normalize;
            joint.Joint.PrismaticJoint.ReferenceAngle = def.ReferenceAngle;
            joint.Joint.PrismaticJoint.Impulse = Vec2.Zero;
            joint.Joint.PrismaticJoint.AxialMass = 0.0f;
            joint.Joint.PrismaticJoint.SpringImpulse = 0.0f;
            joint.Joint.PrismaticJoint.MotorImpulse = 0.0f;
            joint.Joint.PrismaticJoint.LowerImpulse = 0.0f;
            joint.Joint.PrismaticJoint.UpperImpulse = 0.0f;
            joint.Joint.PrismaticJoint.Hertz = def.Hertz;
            joint.Joint.PrismaticJoint.DampingRatio = def.DampingRatio;
            joint.Joint.PrismaticJoint.LowerTranslation = def.LowerTranslation;
            joint.Joint.PrismaticJoint.UpperTranslation = def.UpperTranslation;
            joint.Joint.PrismaticJoint.MaxMotorForce = def.MaxMotorForce;
            joint.Joint.PrismaticJoint.MotorSpeed = def.MotorSpeed;
            joint.Joint.PrismaticJoint.EnableSpring = def.EnableSpring;
            joint.Joint.PrismaticJoint.EnableLimit = def.EnableLimit;
            joint.Joint.PrismaticJoint.EnableMotor = def.EnableMotor;

            // If the joint prevents collisions, then destroy all contacts between attached bodies
            if (def.CollideConnected == false)
            {
                DestroyContactsBetweenBodies(world, bodyA, bodyB);
            }

            JointId jointId = new(joint.JointId + 1, world.WorldId, (ushort)pair.Joint.Revision);
            return jointId;
        }

        public static JointId CreateWeldJoint(WorldId worldId, in WeldJointDef def)
        {
            def.CheckDef();
            World world = World.GetWorldFromId(worldId);

            Debug.Assert(world.Locked == false);

            if (world.Locked)
            {
                return new JointId();
            }

            Body bodyA = Body.GetBodyFullId(world, def.BodyIdA);
            Body bodyB = Body.GetBodyFullId(world, def.BodyIdB);

            JointPair pair = CreateJoint(world, bodyA, bodyB, def.UserData, 1.0f, JointType.WeldJoint, def.CollideConnected);

            JointSim joint = pair.JointSim;
            joint.Type = JointType.WeldJoint;
            joint.LocalOriginAnchorA = def.LocalAnchorA;
            joint.LocalOriginAnchorB = def.LocalAnchorB;

            WeldJoint empty = new();
            joint.Joint.WeldJoint = empty;
            joint.Joint.WeldJoint.ReferenceAngle = def.ReferenceAngle;
            joint.Joint.WeldJoint.LinearHertz = def.LinearHertz;
            joint.Joint.WeldJoint.LinearDampingRatio = def.LinearDampingRatio;
            joint.Joint.WeldJoint.AngularHertz = def.AngularHertz;
            joint.Joint.WeldJoint.AngularDampingRatio = def.AngularDampingRatio;
            joint.Joint.WeldJoint.LinearImpulse = Vec2.Zero;
            joint.Joint.WeldJoint.AngularImpulse = 0.0f;

            // If the joint prevents collisions, then destroy all contacts between attached bodies
            if (def.CollideConnected == false)
            {
                DestroyContactsBetweenBodies(world, bodyA, bodyB);
            }

            JointId jointId = new(joint.JointId + 1, world.WorldId, (ushort)pair.Joint.Revision);
            return jointId;
        }

        public static JointId CreateWheelJoint(WorldId worldId, in WheelJointDef def)

        {
            def.CheckDef();
            World world = World.GetWorldFromId(worldId);

            Debug.Assert(world.Locked == false);

            if (world.Locked)
            {
                return new JointId();
            }

            Body bodyA = Body.GetBodyFullId(world, def.BodyIdA);
            Body bodyB = Body.GetBodyFullId(world, def.BodyIdB);

            JointPair pair = CreateJoint(world, bodyA, bodyB, def.UserData, 1.0f, JointType.WheelJoint, def.CollideConnected);

            JointSim joint = pair.JointSim;
            joint.Type = JointType.WheelJoint;
            joint.LocalOriginAnchorA = def.LocalAnchorA;
            joint.LocalOriginAnchorB = def.LocalAnchorB;

            joint.Joint.WheelJoint = new WheelJoint();
            joint.Joint.WheelJoint.LocalAxisA = def.LocalAxisA.Normalize;
            joint.Joint.WheelJoint.PerpMass = 0.0f;
            joint.Joint.WheelJoint.AxialMass = 0.0f;
            joint.Joint.WheelJoint.MotorImpulse = 0.0f;
            joint.Joint.WheelJoint.LowerImpulse = 0.0f;
            joint.Joint.WheelJoint.UpperImpulse = 0.0f;
            joint.Joint.WheelJoint.LowerTranslation = def.LowerTranslation;
            joint.Joint.WheelJoint.UpperTranslation = def.UpperTranslation;
            joint.Joint.WheelJoint.MaxMotorTorque = def.MaxMotorTorque;
            joint.Joint.WheelJoint.MotorSpeed = def.MotorSpeed;
            joint.Joint.WheelJoint.Hertz = def.Hertz;
            joint.Joint.WheelJoint.DampingRatio = def.DampingRatio;
            joint.Joint.WheelJoint.EnableSpring = def.EnableSpring;
            joint.Joint.WheelJoint.EnableLimit = def.EnableLimit;
            joint.Joint.WheelJoint.EnableMotor = def.EnableMotor;

            // If the joint prevents collisions, then destroy all contacts between attached bodies
            if (def.CollideConnected == false)
            {
                DestroyContactsBetweenBodies(world, bodyA, bodyB);
            }

            JointId jointId = new(joint.JointId + 1, world.WorldId, (ushort)pair.Joint.Revision);
            return jointId;
        }

        public static void DestroyJointInternal(World world, Joint joint, bool wakeBodies)
        {
            int jointId = joint.JointId;

            ref readonly JointEdge edgeA = ref joint.Edges[0];
            ref readonly JointEdge edgeB = ref joint.Edges[1];

            int idA = edgeA.BodyId;
            int idB = edgeB.BodyId;
            Body bodyA = Body.GetBody(world, idA);
            Body bodyB = Body.GetBody(world, idB);

            // Remove from body A
            if (edgeA.PrevKey != Core.NullIndex)
            {
                Joint prevJoint = world.JointArray[(edgeA.PrevKey >> 1)];
                ref JointEdge prevEdge = ref prevJoint.Edges[(edgeA.PrevKey & 1)];
                prevEdge.NextKey = edgeA.NextKey;
            }

            if (edgeA.NextKey != Core.NullIndex)
            {
                Joint nextJoint = world.JointArray[(edgeA.NextKey >> 1)];
                ref JointEdge nextEdge = ref nextJoint.Edges[(edgeA.NextKey & 1)];
                nextEdge.PrevKey = edgeA.PrevKey;
            }

            int edgeKeyA = (jointId << 1) | 0;
            if (bodyA.HeadJointKey == edgeKeyA)
            {
                bodyA.HeadJointKey = edgeA.NextKey;
            }

            bodyA.JointCount -= 1;

            // Remove from body B
            if (edgeB.PrevKey != Core.NullIndex)
            {
                Joint prevJoint = world.JointArray[(edgeB.PrevKey >> 1)];
                ref JointEdge prevEdge = ref prevJoint.Edges[(edgeB.PrevKey & 1)];
                prevEdge.NextKey = edgeB.NextKey;
            }

            if (edgeB.NextKey != Core.NullIndex)
            {
                Joint nextJoint = world.JointArray[(edgeB.NextKey >> 1)];
                ref JointEdge nextEdge = ref nextJoint.Edges[(edgeB.NextKey & 1)];
                nextEdge.PrevKey = edgeB.PrevKey;
            }

            int edgeKeyB = (jointId << 1) | 1;
            if (bodyB.HeadJointKey == edgeKeyB)
            {
                bodyB.HeadJointKey = edgeB.NextKey;
            }

            bodyB.JointCount -= 1;

            if (joint.IslandId != Core.NullIndex)
            {
                Debug.Assert(joint.SetIndex > SolverSetType.DisabledSet);
                Island.UnlinkJoint(world, joint);
            }
            else
            {
                Debug.Assert(joint.SetIndex <= SolverSetType.DisabledSet);
            }

            // Remove joint from solver set that owns it
            int setIndex = joint.SetIndex;
            int localIndex = joint.LocalIndex;

            if (setIndex == SolverSetType.AwakeSet)
            {
                ConstraintGraph.RemoveJointFromGraph(world, joint.Edges[0].BodyId, joint.Edges[1].BodyId, joint.ColorIndex, localIndex);
            }
            else
            {
                SolverSet set = world.SolverSetArray[setIndex];
                int movedIndex = set.Joints.RemoveJoint(localIndex);
                if (movedIndex != Core.NullIndex)
                {
                    // Fix moved joint
                    JointSim movedJointSim = set.Joints.Data[localIndex];
                    int movedId = movedJointSim.JointId;
                    Joint movedJoint = world.JointArray[movedId];
                    Debug.Assert(movedJoint.LocalIndex == movedIndex);
                    movedJoint.LocalIndex = localIndex;
                }
            }

            // Free joint and id (preserve joint revision)
            joint.SetIndex = Core.NullIndex;
            joint.LocalIndex = Core.NullIndex;
            joint.ColorIndex = Core.NullIndex;
            joint.JointId = Core.NullIndex;
            world.JointIdPool.FreeId(jointId);

            if (wakeBodies)
            {
                Body.WakeBody(world, bodyA);
                Body.WakeBody(world, bodyB);
            }

            SolverSet.ValidateSolverSets(world);
        }

        public static void DestroyJoint(JointId jointId)
        {
            World world = World.GetWorld(jointId.World0);
            Debug.Assert(world.Locked == false);

            if (world.Locked)
            {
                return;
            }

            Joint joint = GetJointFullId(world, jointId);

            DestroyJointInternal(world, joint, true);
        }

        public static JointType GetType(JointId jointId)
        {
            World world = World.GetWorld(jointId.World0);
            Joint joint = GetJointFullId(world, jointId);
            return joint.Type;
        }

        public static BodyId GetBodyA(JointId jointId)
        {
            World world = World.GetWorld(jointId.World0);
            Joint joint = GetJointFullId(world, jointId);
            return Body.MakeBodyId(world, joint.Edges[0].BodyId);
        }

        public static BodyId GetBodyB(JointId jointId)
        {
            World world = World.GetWorld(jointId.World0);
            Joint joint = GetJointFullId(world, jointId);
            return Body.MakeBodyId(world, joint.Edges[1].BodyId);
        }

        public static Vec2 GetLocalAnchorA(JointId jointId)
        {
            World world = World.GetWorld(jointId.World0);
            Joint joint = GetJointFullId(world, jointId);
            JointSim jointSim = GetJointSim(world, joint);
            return jointSim.LocalOriginAnchorA;
        }

        public static Vec2 GetLocalAnchorB(JointId jointId)
        {
            World world = World.GetWorld(jointId.World0);
            Joint joint = GetJointFullId(world, jointId);
            JointSim jointSim = GetJointSim(world, joint);
            return jointSim.LocalOriginAnchorB;
        }

        public static void SetCollideConnected(JointId jointId, bool shouldCollide)
        {
            var world = World.GetWorldLocked(jointId.World0);

            Joint joint = GetJointFullId(world, jointId);
            if (joint.CollideConnected == shouldCollide)
            {
                return;
            }

            joint.CollideConnected = shouldCollide;

            Body bodyA = Body.GetBody(world, joint.Edges[0].BodyId);
            Body bodyB = Body.GetBody(world, joint.Edges[1].BodyId);

            if (shouldCollide)
            {
                // need to tell the broad-phase to look for new pairs for one of the
                // two bodies. Pick the one with the fewest shapes.
                int shapeCountA = bodyA.ShapeCount;
                int shapeCountB = bodyB.ShapeCount;

                int shapeId = shapeCountA < shapeCountB ? bodyA.HeadShapeId : bodyB.HeadShapeId;
                while (shapeId != Core.NullIndex)
                {
                    Shape shape = world.ShapeArray[shapeId];

                    if (shape.ProxyKey != Core.NullIndex)
                    {
                        world.BroadPhase.BufferMove(shape.ProxyKey);
                    }

                    shapeId = shape.NextShapeId;
                }
            }
            else
            {
                DestroyContactsBetweenBodies(world, bodyA, bodyB);
            }
        }

        public static bool GetCollideConnected(JointId jointId)
        {
            World world = World.GetWorld(jointId.World0);
            Joint joint = GetJointFullId(world, jointId);
            return joint.CollideConnected;
        }

        public static void SetUserData(JointId jointId, object userData)
        {
            World world = World.GetWorld(jointId.World0);
            Joint joint = GetJointFullId(world, jointId);
            joint.UserData = userData;
        }

        public static object? GetUserData(JointId jointId)
        {
            World world = World.GetWorld(jointId.World0);
            Joint joint = GetJointFullId(world, jointId);
            return joint.UserData;
        }

        public static void WakeBodies(JointId jointId)
        {
            var world = World.GetWorldLocked(jointId.World0);

            Joint joint = GetJointFullId(world, jointId);
            Body bodyA = world.BodyArray[joint.Edges[0].BodyId];
            Body bodyB = world.BodyArray[joint.Edges[1].BodyId];

            Body.WakeBody(world, bodyA);
            Body.WakeBody(world, bodyB);
        }

        public static Vec2 GetConstraintForce(JointId jointId)
        {
            World world = World.GetWorld(jointId.World0);
            Joint joint = GetJointFullId(world, jointId);
            JointSim sim = GetJointSim(world, joint);

            switch (joint.Type)
            {
            case JointType.DistanceJoint:
                return DistanceJointFunc.GetDistanceJointForce(world, sim);

            case JointType.MotorJoint:
                return MotorJointFunc.GetMotorJointForce(world, sim);

            case JointType.MouseJoint:
                return MouseJointFunc.GetMouseJointForce(world, sim);

            case JointType.PrismaticJoint:
                return PrismaticJointFunc.GetPrismaticJointForce(world, sim);

            case JointType.RevoluteJoint:
                return RevoluteJointFunc.GetRevoluteJointForce(world, sim);

            case JointType.WeldJoint:
                return WeldJointFunc.GetWeldJointForce(world, sim);

            case JointType.WheelJoint:
                return WheelJointFunc.GetWheelJointForce(world, sim);

            default:
                Debug.Assert(false);
                return Vec2.Zero;
            }
        }

        public static float GetConstraintTorque(JointId jointId)
        {
            World world = World.GetWorld(jointId.World0);
            Joint joint = GetJointFullId(world, jointId);
            JointSim baseSim = GetJointSim(world, joint);

            switch (joint.Type)
            {
            case JointType.DistanceJoint:
                return 0.0f;

            case JointType.MotorJoint:
                return MotorJointFunc.GetMotorJointTorque(world, baseSim);

            case JointType.MouseJoint:
                return MouseJointFunc.GetMouseJointTorque(world, baseSim);

            case JointType.PrismaticJoint:
                return PrismaticJointFunc.GetPrismaticJointTorque(world, baseSim);

            case JointType.RevoluteJoint:
                return RevoluteJointFunc.GetRevoluteJointTorque(world, baseSim);

            case JointType.WeldJoint:
                return WeldJointFunc.GetWeldJointTorque(world, baseSim);

            case JointType.WheelJoint:
                return WheelJointFunc.GetWheelJointTorque(world, baseSim);

            default:
                Debug.Assert(false);
                return 0.0f;
            }
        }

        public static void PrepareJoint(JointSim joint, StepContext context)
        {
            switch (joint.Type)
            {
            case JointType.DistanceJoint:
                DistanceJointFunc.PrepareDistanceJoint(joint, context);
                break;

            case JointType.MotorJoint:
                MotorJointFunc.PrepareMotorJoint(joint, context);
                break;

            case JointType.MouseJoint:
                MouseJointFunc.PrepareMouseJoint(joint, context);
                break;

            case JointType.PrismaticJoint:
                PrismaticJointFunc.PreparePrismaticJoint(joint, context);
                break;

            case JointType.RevoluteJoint:
                RevoluteJointFunc.PrepareRevoluteJoint(joint, context);
                break;

            case JointType.WeldJoint:
                WeldJointFunc.PrepareWeldJoint(joint, context);
                break;

            case JointType.WheelJoint:
                WheelJointFunc.PrepareWheelJoint(joint, context);
                break;

            default:
                Debug.Assert(false);
                break;
            }
        }

        public static void WarmStartJoint(JointSim joint, StepContext context)
        {
            switch (joint.Type)
            {
            case JointType.DistanceJoint:
                DistanceJointFunc.WarmStartDistanceJoint(joint, context);
                break;

            case JointType.MotorJoint:
                MotorJointFunc.WarmStartMotorJoint(joint, context);
                break;

            case JointType.MouseJoint:
                MouseJointFunc.WarmStartMouseJoint(joint, context);
                break;

            case JointType.PrismaticJoint:
                PrismaticJointFunc.WarmStartPrismaticJoint(joint, context);
                break;

            case JointType.RevoluteJoint:
                RevoluteJointFunc.WarmStartRevoluteJoint(joint, context);
                break;

            case JointType.WeldJoint:
                WeldJointFunc.WarmStartWeldJoint(joint, context);
                break;

            case JointType.WheelJoint:
                WheelJointFunc.WarmStartWheelJoint(joint, context);
                break;

            default:
                Debug.Assert(false);
                break;
            }
        }

        public static void SolveJoint(JointSim joint, StepContext context, bool useBias)
        {
            switch (joint.Type)
            {
            case JointType.DistanceJoint:
                DistanceJointFunc.SolveDistanceJoint(joint, context, useBias);
                break;

            case JointType.MotorJoint:
                MotorJointFunc.SolveMotorJoint(joint, context, useBias);
                break;

            case JointType.MouseJoint:
                MouseJointFunc.SolveMouseJoint(joint, context);
                break;

            case JointType.PrismaticJoint:
                PrismaticJointFunc.SolvePrismaticJoint(joint, context, useBias);
                break;

            case JointType.RevoluteJoint:
                RevoluteJointFunc.SolveRevoluteJoint(joint, context, useBias);
                break;

            case JointType.WeldJoint:
                WeldJointFunc.SolveWeldJoint(joint, context, useBias);
                break;

            case JointType.WheelJoint:
                WheelJointFunc.SolveWheelJoint(joint, context, useBias);
                break;

            default:
                Debug.Assert(false);
                break;
            }
        }

        public static void PrepareOverflowJoints(StepContext context)
        {
            World world = context.World;
            ConstraintGraph graph = context.Graph;
            var joints = graph.Colors[Core.OverflowIndex].Joints.Data;
            int jointCount = graph.Colors[Core.OverflowIndex].Joints.Count;

            for (int i = 0; i < jointCount; ++i)
            {
                JointSim joint = joints[i];
                PrepareJoint(joint, context);
            }
        }

        public static void WarmStartOverflowJoints(StepContext context)
        {
            World world = context.World;
            ConstraintGraph graph = context.Graph;
            var joints = graph.Colors[Core.OverflowIndex].Joints.Data;
            int jointCount = graph.Colors[Core.OverflowIndex].Joints.Count;

            for (int i = 0; i < jointCount; ++i)
            {
                JointSim joint = joints[i];
                WarmStartJoint(joint, context);
            }
        }

        public static void SolveOverflowJoints(StepContext context, bool useBias)
        {
            World world = context.World;
            ConstraintGraph graph = context.Graph;
            var joints = graph.Colors[Core.OverflowIndex].Joints.Data;
            int jointCount = graph.Colors[Core.OverflowIndex].Joints.Count;

            for (int i = 0; i < jointCount; ++i)
            {
                JointSim joint = joints[i];
                SolveJoint(joint, context, useBias);
            }
        }

        public static void DrawJoint(DebugDrawBase draw, World world, Joint joint)
        {
            Body bodyA = Body.GetBody(world, joint.Edges[0].BodyId);
            Body bodyB = Body.GetBody(world, joint.Edges[1].BodyId);
            if (bodyA.SetIndex == SolverSetType.DisabledSet || bodyB.SetIndex == SolverSetType.DisabledSet)
            {
                return;
            }

            JointSim jointSim = GetJointSim(world, joint);

            Transform transformA = Body.GetBodyTransformQuick(world, bodyA);
            Transform transformB = Body.GetBodyTransformQuick(world, bodyB);
            Vec2 pA = B2Math.TransformPoint(transformA, jointSim.LocalOriginAnchorA);
            Vec2 pB = B2Math.TransformPoint(transformB, jointSim.LocalOriginAnchorB);

            B2HexColor color = B2HexColor.DarkSeaGreen;

            switch (joint.Type)
            {
            case JointType.DistanceJoint:
                DistanceJointFunc.DrawDistanceJoint(draw, jointSim, transformA, transformB);
                break;

            case JointType.MouseJoint:
            {
                Vec2 target = jointSim.Joint.MouseJoint.TargetA;

                B2HexColor c1 = B2HexColor.Green;
                draw.DrawPoint(target, 4.0f, c1, draw.Context);
                draw.DrawPoint(pB, 4.0f, c1, draw.Context);

                B2HexColor c2 = B2HexColor.Gray8;
                draw.DrawSegment(target, pB, c2, draw.Context);
            }
                break;

            case JointType.PrismaticJoint:
                PrismaticJointFunc.DrawPrismaticJoint(draw, jointSim, transformA, transformB);
                break;

            case JointType.RevoluteJoint:
                RevoluteJointFunc.DrawRevoluteJoint(draw, jointSim, transformA, transformB, joint.DrawSize);
                break;

            case JointType.WheelJoint:
                WheelJointFunc.DrawWheelJoint(draw, jointSim, transformA, transformB);
                break;

            default:
                draw.DrawSegment(transformA.P, pA, color, draw.Context);
                draw.DrawSegment(pA, pB, color, draw.Context);
                draw.DrawSegment(transformB.P, pB, color, draw.Context);
                break;
            }

            if (draw.DrawGraphColors)
            {
                var colors = new B2HexColor[Core.GraphColorCount]
                {
                    B2HexColor.Red, B2HexColor.Orange, B2HexColor.Yellow, B2HexColor.Green,
                    B2HexColor.Cyan, B2HexColor.Blue, B2HexColor.Violet, B2HexColor.Pink,
                    B2HexColor.Chocolate, B2HexColor.Goldenrod, B2HexColor.Coral, B2HexColor.Black
                };

                int colorIndex = joint.ColorIndex;
                if (colorIndex != Core.NullIndex)
                {
                    Vec2 p = B2Math.Lerp(pA, pB, 0.5f);
                    draw.DrawPoint(p, 5.0f, colors[colorIndex], draw.Context);
                }
            }
        }
    }
}