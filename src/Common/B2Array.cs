using System;
using System.Diagnostics;

namespace Box2DSharp
{
    public class B2Array<T> : IDisposable
    {
        public T[] Data;

        public int Count;

        public int Capacity;

        public Span<T> Span => Data.AsSpan(0, Capacity);

        public Memory<T> Memory => Data.AsMemory(0, Capacity);

        public ref T this[int index] => ref Span[index];

        public ref T this[Index index] => ref Span[index];

        public static implicit operator Span<T>(B2Array<T> arr)
        {
            return arr.Span;
        }

        public static implicit operator Memory<T>(B2Array<T> arr)
        {
            return arr.Memory;
        }

        public B2Array(int capacity = Core.InitialCapacity)
        {
            Capacity = capacity;
            Data = B2ArrayPool<T>.Shared.Rent(Capacity);
            Count = 0;
        }

        public void EnsureCapacity()
        {
            if (Capacity == 0)
            {
                Debug.Assert(Count == 0);
                Capacity = Core.InitialCapacity;
                Data = B2ArrayPool<T>.Shared.Rent(Capacity);
                Count = 0;
            }
            else if (Count == Capacity)
            {
                Capacity = 2 * Capacity;
                B2ArrayPool<T>.Shared.Resize(ref Data, Capacity);
            }
        }

        public void Push(T value)
        {
            if (Count == Capacity)
            {
                Grow();
            }

            Debug.Assert(Count < Capacity);
            Span[Count++] = value;
        }

        public void RemoveSwap(int index)
        {
            Debug.Assert(0 <= index && index < Count);
            Span[index] = Span[Count - 1];
            Count--;
        }

        public T Last()
        {
            return Span[Count - 1];
        }

        public void Pop()
        {
            Debug.Assert(0 < Count);
            Count--;
        }

        public void Grow()
        {
            Debug.Assert(Capacity == Count);

            // grow by 50%
            var newCapacity = Capacity + (Capacity >> 1);
            newCapacity = newCapacity >= 2 ? newCapacity : 2;

            B2ArrayPool<T>.Shared.Resize(ref Data, newCapacity);
            Capacity = newCapacity;
        }

        public void Resize(int count)
        {
            if (Capacity >= count)
            {
                Count = count;
                return;
            }

            // grow by 50%
            var newCapacity = count + (count >> 1);
            newCapacity = newCapacity >= 2 ? newCapacity : 2;

            B2ArrayPool<T>.Shared.Resize(ref Data, newCapacity);
            Capacity = newCapacity;
            Count = count;
        }

        public void Clear()
        {
            Count = 0;
        }

        public void Dispose()
        {
            B2ArrayPool<T>.Shared.Return(Data);
            Data = null!;
        }
    }
}