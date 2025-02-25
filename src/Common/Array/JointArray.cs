using System.Diagnostics;

namespace Box2DSharp
{
    public class JointArray : B2Array<JointSim>
    {
        public JointArray()
        { }

        public JointArray(int capacity)
            : base(capacity)
        { }

        public JointSim AddJoint()
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

        public int RemoveJoint(int index)
        {
            Debug.Assert(0 <= index && index < Count);
            if (index < Count - 1)
            {
                // 把最后一个有效对象和待删除对象交换位置，然后重置待删除对象的状态，并返回交换后的空位的索引
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