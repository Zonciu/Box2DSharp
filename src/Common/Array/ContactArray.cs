using System.Diagnostics;

namespace Box2DSharp
{
    public class ContactArray : B2Array<ContactSim>
    {
        public ContactArray()
        { }

        public ContactArray(int capacity)
            : base(capacity)
        { }

        public override string ToString()
        {
            return $"{Count}/{Capacity}";
        }

        public ContactSim AddContact()
        {
            EnsureCapacity();
            if (Data[Count] == null!)
            {
                Data[Count] = new();
            }

            // 申请时返回尾部第一个元素
            var element = Data[Count];
            Count += 1;
            return element;
        }

        public int RemoveContact(int index)
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