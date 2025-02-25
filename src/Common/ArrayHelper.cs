using System;

namespace Box2DSharp
{
    public static class ArrayHelper
    {
        public static void FillFromPool<T>(T[] array, int start, int length)
            where T : class, new()
        {
            var span = array.AsSpan(start, length);
            for (var i = 0; i < span.Length; i++)
            {
                span[i] = B2ObjectPool<T>.Shared.Get();
            }
        }

        public static void ReturnToPool<T>(T[] array, int start, int length)
            where T : class, new()
        {
            var span = array.AsSpan(start, length);
            foreach (var t in span)
            {
                B2ObjectPool<T>.Shared.Return(t);
            }
        }
    }
}