using System.Runtime.CompilerServices;
using System.Threading;

namespace Box2DSharp
{
    public static class InterlockedHelper
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint Exchange(ref uint location1, uint value) =>
            (uint)Interlocked.Exchange(ref Unsafe.As<uint, int>(ref location1), (int)value);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint CompareExchange(ref uint location1, uint value, uint comparand) =>
            (uint)Interlocked.CompareExchange(ref Unsafe.As<uint, int>(ref location1), (int)value, (int)comparand);
    }
}