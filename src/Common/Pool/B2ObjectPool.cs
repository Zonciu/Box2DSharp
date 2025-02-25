using System;
using System.Runtime.CompilerServices;
using System.Threading;

namespace Box2DSharp
{
    public class B2ObjectPool<T>
        where T : class, new()
    {
        public static readonly B2ObjectPool<T> Shared = new();

        private T[] _objects;

        private int _total;

        private int _capacity;

        private int _lock = Unlocked;

        private const int Locked = 1;

        private const int Unlocked = 0;

        public B2ObjectPool(int capacity = 256)
        {
            _capacity = capacity;
            _objects = new T[_capacity];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T Get()
        {
            while (Interlocked.CompareExchange(ref _lock, Locked, Unlocked) == Locked)
            {
                // spin wait
            }

            if (_total > 0)
            {
                --_total;
                var item = _objects[_total];
                _objects[_total] = null!;
                Interlocked.Exchange(ref _lock, Unlocked);
                return item;
            }

            Interlocked.Exchange(ref _lock, Unlocked);
            return new T();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Return(T item)
        {
            while (Interlocked.CompareExchange(ref _lock, Locked, Unlocked) == Locked)
            {
                // spin wait
            }

            if (_total >= _capacity)
            {
                _capacity *= 2;
                Array.Resize(ref _objects, _capacity);
            }

            _objects[_total] = item;
            ++_total;

            Interlocked.Exchange(ref _lock, Unlocked);
        }

        /// <summary>
        /// Return array objects
        /// </summary>
        /// <param name="items"></param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Return(Span<T> items)
        {
            while (Interlocked.CompareExchange(ref _lock, Locked, Unlocked) == Locked)
            {
                // spin wait
            }

            foreach (var t in items)
            {
                if (_total >= _capacity)
                {
                    _capacity *= 2;
                    Array.Resize(ref _objects, _capacity);
                }

                _objects[_total] = t;
                ++_total;
            }

            Interlocked.Exchange(ref _lock, Unlocked);
        }
    }
}