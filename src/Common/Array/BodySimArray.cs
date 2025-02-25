using System.Diagnostics;

namespace Box2DSharp
{
    public class BodySimArray : B2Array<BodySim>
    {
        public BodySimArray()
        { }

        public BodySimArray(int capacity)
            : base(capacity)
        { }

        public BodySim AddBodySim()
        {
            EnsureCapacity();

            if (Data[Count] == null!)
            {
                Data[Count] = new();
            }

            var element = Data[Count];
            Count += 1;
            return element;
        }

        // Returns the index of the element moved into the empty slot (or B2_NULL_INDEX)
        public int RemoveBodySim(int index)
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