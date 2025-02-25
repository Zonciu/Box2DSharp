using System.Diagnostics;

namespace Box2DSharp
{
    public class IslandArray : B2Array<IslandSim>
    {
        public IslandArray()
        { }

        public IslandArray(int capacity)
            : base(capacity)
        { }

        public IslandSim AddIsland()
        {
            EnsureCapacity();
            if (Data[Count] == null!)
            {
                Data[Count] = new();
            }

            IslandSim element = Data[Count];

            element.IslandId = Core.NullIndex;
            Count += 1;
            return element;
        }

        public int RemoveIsland(int index)
        {
            Debug.Assert(0 <= index && index < Count);
            if (index < Count - 1)
            {
                // 把最后一个有效对象和待删除对象交换位置，然后重置待删除对象的状态
                var removed = Count - 1;
                (Data[index], Data[removed]) = (Data[removed], Data[index]);
                Data[removed].Reset();
                Count -= 1;
                return removed;
            }

            Count -= 1;
            return Core.NullIndex;
        }
    }
}