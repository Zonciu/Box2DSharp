namespace Box2DSharp
{
    public class TaskContext
    {
        // These bits align with the b2ConstraintGraph::contactBlocks and signal a change in contact status
        public BitSet ContactStateBitSet = new(1024);

        // Used to track bodies with shapes that have enlarged AABBs. This avoids having a bit array
        // that is very large when there are many static shapes.
        public BitSet EnlargedSimBitSet = new(256);

        // Used to put islands to sleep
        public BitSet AwakeIslandBitSet = new(256);

        // Per worker split island candidate
        public float SplitSleepTime;

        public int SplitIslandId;
    }
}