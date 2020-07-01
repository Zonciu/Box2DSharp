using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Box2DSharp.Dynamics.Contacts
{
    public class ContactPool<T>
        where T : Contact, new()
    {
        private readonly Queue<T> _objects;

        public ContactPool(int capacity = 256)
        {
            _objects = new Queue<T>(capacity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T Get()
        {
            return _objects.Count > 0 ? _objects.Dequeue() : new T();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Return(T item)
        {
            item.Reset();
            _objects.Enqueue(item);
        }
    }
}