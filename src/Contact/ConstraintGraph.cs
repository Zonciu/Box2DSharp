using System;
using System.Diagnostics;

namespace Box2DSharp
{
    /// <summary>
    /// 着色约束图
    /// </summary>
    public class ConstraintGraph
    {
        /// <summary>
        /// including overflow at the end 
        /// </summary>
        public GraphColor[] Colors = new GraphColor[Core.GraphColorCount];

        static ConstraintGraph()
        {
            Debug.Assert(Core.GraphColorCount == 12, "graph color count assumed to be 12");
        }

        public ConstraintGraph(int bodyCapacity)
        {
            Debug.Assert(Core.GraphColorCount >= 2, "must have at least two constraint graph colors");
            Debug.Assert(Core.OverflowIndex == Core.GraphColorCount - 1, "bad over flow index");

            bodyCapacity = Math.Max(bodyCapacity, 8);

            // Initialize graph color bit set.
            // No bitset for overflow color.
            for (var i = 0; i < Core.OverflowIndex; ++i)
            {
                Colors[i] = new GraphColor(bodyCapacity);
            }

            Colors[Core.OverflowIndex] = new GraphColor();
        }

        // todo Rename Dispose
        public void DestroyGraph()
        {
            for (var i = 0; i < Core.GraphColorCount; ++i)
            {
                var color = Colors[i];

                // The bit set should never be used on the overflow color
                Debug.Assert(i != Core.OverflowIndex || color.BodySet == null);

                color.BodySet = null!;
                color.Contacts = null!;
                color.Joints = null!;
            }
        }

        // Contacts are always created as non-touching. They get cloned into the constraint
        // graph once they are found to be touching.
        // todo maybe kinematic bodies should not go into graph
        public static void AddContactToGraph(World world, ContactSim contactSim, Contact contact)
        {
            Debug.Assert(contactSim.Manifold.PointCount > 0);
            Debug.Assert(contactSim.SimFlags.IsSet(ContactSimFlags.TouchingFlag));
            Debug.Assert(contact.Flags.IsSet(ContactFlags.ContactTouchingFlag));

            ConstraintGraph graph = world.ConstraintGraph;
            int colorIndex = Core.OverflowIndex;

            int bodyIdA = contact.Edges[0].BodyId;
            int bodyIdB = contact.Edges[1].BodyId;
            world.BodyArray.CheckIndex(bodyIdA);
            world.BodyArray.CheckIndex(bodyIdB);

            Body bodyA = world.BodyArray[bodyIdA];
            Body bodyB = world.BodyArray[bodyIdB];
            bool staticA = bodyA.SetIndex == SolverSetType.StaticSet;
            bool staticB = bodyB.SetIndex == SolverSetType.StaticSet;
            Debug.Assert(staticA == false || staticB == false);

            if (Core.ForceOverflow)
            {
                if (staticA == false && staticB == false)
                {
                    for (int i = 0; i < Core.OverflowIndex; ++i)
                    {
                        GraphColor color = graph.Colors[i];
                        if (color.BodySet.GetBit(bodyIdA) || color.BodySet.GetBit(bodyIdB))
                        {
                            continue;
                        }

                        color.BodySet.SetBitGrow(bodyIdA);
                        color.BodySet.SetBitGrow(bodyIdB);
                        colorIndex = i;
                        break;
                    }
                }
                else if (staticA == false)
                {
                    // No static contacts in color 0
                    for (int i = 1; i < Core.OverflowIndex; ++i)
                    {
                        GraphColor color = graph.Colors[i];
                        if (color.BodySet.GetBit(bodyIdA))
                        {
                            continue;
                        }

                        color.BodySet.SetBitGrow(bodyIdA);
                        colorIndex = i;
                        break;
                    }
                }
                else if (staticB == false)
                {
                    // No static contacts in color 0
                    for (int i = 1; i < Core.OverflowIndex; ++i)
                    {
                        GraphColor color = graph.Colors[i];
                        if (color.BodySet.GetBit(bodyIdB))
                        {
                            continue;
                        }

                        color.BodySet.SetBitGrow(bodyIdB);
                        colorIndex = i;
                        break;
                    }
                }
            }

            {
                GraphColor color = graph.Colors[colorIndex];
                contact.ColorIndex = colorIndex;
                contact.LocalIndex = color.Contacts.Count;
                ContactSim newContact = color.Contacts.AddContact();
                contactSim.CopyTo(newContact);

                // todo perhaps skip this if the contact is already awake

                if (staticA)
                {
                    newContact.BodySimIndexA = Core.NullIndex;
                    newContact.InvMassA = 0.0f;
                    newContact.InvIA = 0.0f;
                }
                else
                {
                    Debug.Assert(bodyA.SetIndex == SolverSetType.AwakeSet);
                    SolverSet awakeSet = world.SolverSetArray[SolverSetType.AwakeSet];

                    int localIndex = bodyA.LocalIndex;
                    Debug.Assert(0 <= localIndex && localIndex < awakeSet.Sims.Count);
                    newContact.BodySimIndexA = localIndex;

                    BodySim bodySimA = awakeSet.Sims.Data[localIndex];
                    newContact.InvMassA = bodySimA.InvMass;
                    newContact.InvIA = bodySimA.InvInertia;
                }

                if (staticB)
                {
                    newContact.BodySimIndexB = Core.NullIndex;
                    newContact.InvMassB = 0.0f;
                    newContact.InvIB = 0.0f;
                }
                else
                {
                    Debug.Assert(bodyB.SetIndex == SolverSetType.AwakeSet);
                    SolverSet awakeSet = world.SolverSetArray[SolverSetType.AwakeSet];

                    int localIndex = bodyB.LocalIndex;
                    Debug.Assert(0 <= localIndex && localIndex < awakeSet.Sims.Count);
                    newContact.BodySimIndexB = localIndex;

                    BodySim bodySimB = awakeSet.Sims.Data[localIndex];
                    newContact.InvMassB = bodySimB.InvMass;
                    newContact.InvIB = bodySimB.InvInertia;
                }
            }
        }

        public static void RemoveContactFromGraph(World world, int bodyIdA, int bodyIdB, int colorIndex, int localIndex)
        {
            ConstraintGraph graph = world.ConstraintGraph;

            Debug.Assert(0 <= colorIndex && colorIndex < Core.GraphColorCount);
            GraphColor color = graph.Colors[colorIndex];

            if (colorIndex != Core.OverflowIndex)
            {
                // might clear a bit for a static body, but this has no effect
                color.BodySet.ClearBit(bodyIdA);
                color.BodySet.ClearBit(bodyIdB);
            }

            int movedIndex = color.Contacts.RemoveContact(localIndex);
            if (movedIndex != Core.NullIndex)
            {
                // Fix index on swapped contact
                ContactSim movedContactSim = color.Contacts.Data[localIndex];

                // Fix moved contact
                int movedId = movedContactSim.ContactId;
                world.ContactArray.CheckIndex(movedId);
                Contact movedContact = world.ContactArray[movedId];
                Debug.Assert(movedContact.SetIndex == SolverSetType.AwakeSet);
                Debug.Assert(movedContact.ColorIndex == colorIndex);
                Debug.Assert(movedContact.LocalIndex == movedIndex);
                movedContact.LocalIndex = localIndex;
            }
        }

        public static int AssignJointColor(ConstraintGraph graph, int bodyIdA, int bodyIdB, bool staticA, bool staticB)
        {
            Debug.Assert(staticA == false || staticB == false);

            if (Core.ForceOverflow)
            {
                if (staticA == false && staticB == false)
                {
                    for (int i = 0; i < Core.OverflowIndex; ++i)
                    {
                        GraphColor color = graph.Colors[i];
                        if (color.BodySet.GetBit(bodyIdA) || color.BodySet.GetBit(bodyIdB))
                        {
                            continue;
                        }

                        color.BodySet.SetBitGrow(bodyIdA);
                        color.BodySet.SetBitGrow(bodyIdB);
                        return i;
                    }
                }
                else if (staticA == false)
                {
                    for (int i = 0; i < Core.OverflowIndex; ++i)
                    {
                        GraphColor color = graph.Colors[i];
                        if (color.BodySet.GetBit(bodyIdA))
                        {
                            continue;
                        }

                        color.BodySet.SetBitGrow(bodyIdA);
                        return i;
                    }
                }
                else if (staticB == false)
                {
                    for (int i = 0; i < Core.OverflowIndex; ++i)
                    {
                        GraphColor color = graph.Colors[i];
                        if (color.BodySet.GetBit(bodyIdB))
                        {
                            continue;
                        }

                        color.BodySet.SetBitGrow(bodyIdB);
                        return i;
                    }
                }
            }

            return Core.OverflowIndex;
        }

        public static JointSim CreateJointInGraph(World world, Joint joint)
        {
            var graph = world.ConstraintGraph;

            int bodyIdA = joint.Edges[0].BodyId;
            int bodyIdB = joint.Edges[1].BodyId;
            world.BodyArray.CheckIndex(bodyIdA);
            world.BodyArray.CheckIndex(bodyIdB);

            Body bodyA = world.BodyArray[bodyIdA];
            Body bodyB = world.BodyArray[bodyIdB];
            bool staticA = bodyA.SetIndex == SolverSetType.StaticSet;
            bool staticB = bodyB.SetIndex == SolverSetType.StaticSet;

            int colorIndex = AssignJointColor(graph, bodyIdA, bodyIdB, staticA, staticB);

            JointSim jointSim = graph.Colors[colorIndex].Joints.AddJoint();
            joint.ColorIndex = colorIndex;
            joint.LocalIndex = graph.Colors[colorIndex].Joints.Count - 1;
            return jointSim;
        }

        public static void AddJointToGraph(World world, JointSim jointSim, Joint joint)
        {
            JointSim jointDst = CreateJointInGraph(world, joint);
            jointSim.CopyTo(jointDst);
        }

        public static void RemoveJointFromGraph(World world, int bodyIdA, int bodyIdB, int colorIndex, int localIndex)
        {
            var graph = world.ConstraintGraph;

            Debug.Assert(0 <= colorIndex && colorIndex < Core.GraphColorCount);
            GraphColor color = graph.Colors[colorIndex];

            if (colorIndex != Core.OverflowIndex)
            {
                // May clear static bodies, no effect
                color.BodySet.ClearBit(bodyIdA);
                color.BodySet.ClearBit(bodyIdB);
            }

            int movedIndex = color.Joints.RemoveJoint(localIndex);
            if (movedIndex != Core.NullIndex)
            {
                // Fix moved joint
                JointSim movedJointSim = color.Joints.Data[localIndex];
                int movedId = movedJointSim.JointId;
                world.JointArray.CheckIndex(movedId);
                Joint movedJoint = world.JointArray[movedId];
                Debug.Assert(movedJoint.SetIndex == SolverSetType.AwakeSet);
                Debug.Assert(movedJoint.ColorIndex == colorIndex);
                Debug.Assert(movedJoint.LocalIndex == movedIndex);
                movedJoint.LocalIndex = localIndex;
            }
        }
    }
}