using System;

namespace Box2DSharp
{
    public interface IFixedArray<T>
        where T : unmanaged
    {
        public int Length { get; }

        public Span<T> Span { get; }

        public ref T this[int index] { get; }
    }
}