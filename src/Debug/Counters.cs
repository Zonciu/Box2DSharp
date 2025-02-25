namespace Box2DSharp
{
    /// <summary>
    /// Counters that give details of the simulation size.
    /// </summary>
    public class Counters
    {
        public int StaticBodyCount;

        public int BodyCount;

        public int ShapeCount;

        public int ContactCount;

        public int JointCount;

        public int IslandCount;

        public int StackUsed;

        public int StaticTreeHeight;

        public int TreeHeight;

        public int ByteCount;

        public int TaskCount;

        public int[] ColorCounts = new int[Core.GraphColorCount]; // [12]
    }
}