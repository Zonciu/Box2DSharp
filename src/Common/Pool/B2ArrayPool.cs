using System;
using System.Buffers;
using System.Threading;

namespace Box2DSharp
{
    public class B2ArrayPool<T>
    {
        public static readonly B2ArrayPool<T> Shared = new();

        private readonly ArrayPool<T> _pool = ArrayPool<T>.Shared;

        private int _lock = Unlocked;

        private const int Locked = 1;

        private const int Unlocked = 0;

        public T[] Rent(int length)
        {
            while (Interlocked.CompareExchange(ref _lock, Locked, Unlocked) == Locked)
            {
                // spin wait
            }

            var array = _pool.Rent(length);
            Interlocked.Exchange(ref _lock, Unlocked);
            return array;
        }

        public void Return(T[] item, bool clear = true)
        {
            if (item == null!)
            {
                Interlocked.Exchange(ref _lock, Unlocked);
                return;
            }

            while (Interlocked.CompareExchange(ref _lock, Locked, Unlocked) == Locked)
            {
                // spin wait
            }

            _pool.Return(item, clear);
            Interlocked.Exchange(ref _lock, Unlocked);
        }

        /// <summary>
        /// Rent a new array and copy data
        /// </summary>
        /// <param name="array"></param>
        /// <param name="newSize"></param>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        public void Resize(ref T[] array, int newSize)
        {
            if (newSize < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(newSize));
            }

            while (Interlocked.CompareExchange(ref _lock, Locked, Unlocked) == Locked)
            {
                // spin wait
            }

            var oldArray = array;
            if (oldArray == null!)
            {
                array = Rent(newSize);
            }
            else
            {
                if (oldArray.Length >= newSize)
                {
                    Interlocked.Exchange(ref _lock, Unlocked);
                    return;
                }

                var array2 = _pool.Rent(newSize);
                oldArray.AsSpan().CopyTo(array2);
                array = array2;
                _pool.Return(oldArray, true);
                Interlocked.Exchange(ref _lock, Unlocked);
            }
        }
    }
}