using System;
using System.Buffers;
using System.Collections.Concurrent;
using System.Numerics;

namespace Box2DSharp.Common
{
    internal class ObjectPool<T>
        where T : new()
    {
        private readonly IPooledObjectPolicy<T> _policy;

        private readonly ConcurrentBag<T> _objects;

        private readonly int _maximumRetained;

        public static ObjectPool<T> Shared = new ObjectPool<T>(new DefaultPoolPolicy<T>());

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

    public interface IPooledObjectPolicy<T>
    {
        T Create();

        bool Return(T obj);
    }

    public class ActionPoolPolicy<T> : IPooledObjectPolicy<T>
        where T : new()
    {
        private readonly Action<T> _getAction;

        private readonly Func<T, bool> _returnAction;

        public ActionPoolPolicy(Action<T> getAction = null, Func<T, bool> returnAction = null)
        {
            _getAction = getAction;
            _returnAction = returnAction;
        }

        public T Create()
        {
            var obj = new T();
            _getAction?.Invoke(obj);
            return obj;
        }

        public bool Return(T obj)
        {
            return _returnAction == null || _returnAction.Invoke(obj);
        }
    }

    public class DefaultPoolPolicy<T> : IPooledObjectPolicy<T>
        where T : new()
    {
        public T Create()
        {
            return new T();
        }

        public bool Return(T obj)
        {
            return true;
        }
    }
}