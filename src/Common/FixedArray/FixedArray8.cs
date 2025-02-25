using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DSharp
{
    [StructLayout(LayoutKind.Sequential)]
    public struct FixedArray8<T> : IFixedArray<T>
        where T : unmanaged
    {
        public T V0;

        public T V1;

        public T V2;

        public T V3;

        public T V4;

        public T V5;

        public T V6;

        public T V7;

        public int Length => 8;

        public FixedArray8(T v0, T v1, T v2, T v3, T v4, T v5, T v6, T v7)
        {
            V0 = v0;
            V1 = v1;
            V2 = v2;
            V3 = v3;
            V4 = v4;
            V5 = v5;
            V6 = v6;
            V7 = v7;
        }

        public FixedArray8(IEnumerable<T> enumerable)
            : this()
        {
            var i = 0;
            foreach (var x1 in enumerable)
            {
                this[i++] = x1;
            }
        }

        public static implicit operator Span<T>(FixedArray8<T> v)
        {
            return v.Span;
        }

        public static implicit operator ReadOnlySpan<T>(FixedArray8<T> v)
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
            return $"({V0},{V1},{V2},{V3},{V4},{V5},{V6},{V7})";
        }
    }
}