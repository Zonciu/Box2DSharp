using System;
using System.Runtime.CompilerServices;
using System.Threading;

namespace Box2DSharp.Dynamics.Contacts
{
    public class ContactPool<T>
        where T : Contact, new()
    {
        private T[] _objects;

        private int _total;

        private int _capacity;

        private int _lock = Unlocked;

        private const int Locked = 1;

        private const int Unlocked = 0;

        public ContactPool(int capacity = 256)
        {
            _capacity = capacity;
            _objects = new T[_capacity];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T Get()
        {
            while (Interlocked.CompareExchange(ref _lock, Locked, Unlocked) == Locked)
            { }

            if (_total > 0)
            {
                --_total;
                var item = _objects[_total];
                _objects[_total] = null;
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
            { }

            item.Reset();
            if (_total >= _capacity)
            {
                _capacity *= 2;
                Array.Resize(ref _objects, _capacity);
            }

            _objects[_total] = item;
            ++_total;

            Interlocked.Exchange(ref _lock, Unlocked);
        }
    }
}