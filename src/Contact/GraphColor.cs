using System;

namespace Box2DSharp
{
    /// <summary>
    /// This holds constraints that cannot fit the graph color limit. This happens when a single dynamic body
    /// is touching many other bodies.
    /// </summary>
    public class GraphColor
    {
        /// <summary>
        /// This bitset is indexed by bodyId so this is over-sized to encompass static bodies
        /// however I never traverse these bits or use the bit count for anything
        /// This bitset is unused on the overflow color.
        /// </summary>
        public BitSet BodySet;

        /// <summary>
        /// cache friendly arrays
        /// </summary>
        public ContactArray Contacts;

        public JointArray Joints;

        public Memory<ContactConstraintSIMD> SimdConstraints;

        public Memory<ContactConstraint> OverflowConstraints;

        // transient

        public GraphColor()
        {
            Contacts = new();
            Joints = new();
            BodySet = null!;
        }

        public GraphColor(int bodyCapacity)
        {
            Contacts = new();
            Joints = new();
            BodySet = new(bodyCapacity);
            BodySet.SetBitCountAndClear(bodyCapacity);
        }
    }
}