using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DSharp
{
    [StructLayout(LayoutKind.Sequential)]
    public struct FixedArray3<T> : IFixedArray<T>
        where T : unmanaged
    {
        public T V0;

        public T V1;

        public T V2;

        public int Length => 3;

        public FixedArray3(T v0, T v1, T v2)
        {
            V0 = v0;
            V1 = v1;
            V2 = v2;
        }

        public static implicit operator Span<T>(FixedArray3<T> v)
        {
            return v.Span;
        }

        public static implicit operator ReadOnlySpan<T>(FixedArray3<T> v)
        {
            return v.Span;
        }

        public Span<T> Span
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => MemoryMarshal.CreateSpan(ref V0, Length);
        }

        public ref T this[int index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => ref MemoryMarshal.CreateSpan(ref V0, Length)[index];
        }

        public override string ToString()
        {
            return $"({V0},{V1},{V2})";
        }
    }
}