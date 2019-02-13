using System;
using System.Collections.Concurrent;

namespace Box2DSharp.Common
{
    internal class ObjectPool<T> : IObjectPool
    {
        private readonly IPooledObjectPolicy<T> _policy;

        private readonly ConcurrentBag<T> _objects;

        private readonly int _maximumRetained;

        public ObjectPool(IPooledObjectPolicy<T> policy) : this(policy, Environment.ProcessorCount * 2)
        { }

        public ObjectPool(IPooledObjectPolicy<T> policy, int maximumRetained)
        {
            _policy = policy;
            _objects = new ConcurrentBag<T>();
            _maximumRetained = maximumRetained;
        }

        public T Get()
        {
            return _objects.TryTake(out var item) ? item : _policy.Create();
        }

        public void Return(T item)
        {
            if (_policy.Return(item) && _objects.Count < _maximumRetained)
            {
                _objects.Add(item);
            }
        }
    }

    internal interface IObjectPool
    { }

    public interface IPooledObjectPolicy<T>
    {
        T Create();

        bool Return(T obj);
    }
}