using System.Diagnostics;

namespace Box2DSharp
{
    public class BodyStateArray : B2Array<BodyState>
    {
        public ref BodyState AddBodyState()
        {
            EnsureCapacity();

            ref var element = ref Data[Count];
            element.Reset();
            Count += 1;
            return ref element;
        }

        public BodyStateArray()
        { }

        public BodyStateArray(int capacity)
            : base(capacity)
        { }

        public int RemoveBodyState(int index)
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