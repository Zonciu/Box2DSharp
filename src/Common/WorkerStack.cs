using System;
using System.Threading;

namespace Box2DSharp
{
    public class WorkerStack
    {
        public int[] Data;

        public int Count;

        public int Capacity;

        private int _lockState = Unlocked;

        private const int Locked = 1;

        private const int Unlocked = 0;

        public WorkerStack(int capacity = 8)
        {
            Capacity = capacity;
            Data = new int[capacity];
            Reset();
        }

        public void Push(int value)
        {
            while (Interlocked.CompareExchange(ref _lockState, Locked, Unlocked) != Unlocked)
            { }

            if (Count == Capacity)
            {
                return;
            }

            Data[Count] = value;
            Count++;
            Interlocked.Exchange(ref _lockState, Unlocked);
        }

        public bool TryPop(out int value)
        {
            while (Interlocked.CompareExchange(ref _lockState, Locked, Unlocked) != Unlocked)
            { }

            if (Count == 0)
            {
                value = default!;
                Interlocked.Exchange(ref _lockState, Unlocked);
                return false;
            }

            value = Data[Count - 1];
            Count--;
            Interlocked.Exchange(ref _lockState, Unlocked);
            return true;
        }

        public void Reset()
        {
            while (Interlocked.CompareExchange(ref _lockState, Locked, Unlocked) != Unlocked)
            { }

            Count = Capacity;
            for (var i = 0; i < Data.Length; i++)
            {
                Data[i] = i;
            }

            Interlocked.Exchange(ref _lockState, Unlocked);
        }
    }
}