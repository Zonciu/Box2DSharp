namespace Box2DSharp
{
    public struct ChainShape
    {
        public int Id;

        public int BodyId;

        public int NextChainId;

        public int[] ShapeIndices;

        public int Count;

        public ushort Revision;
    }
}