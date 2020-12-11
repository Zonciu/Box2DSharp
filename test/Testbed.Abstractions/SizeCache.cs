using System.Runtime.InteropServices;

namespace Testbed.Abstractions
{
    public static class SizeCache<T>
    {
        public static readonly int Size = Marshal.SizeOf<T>();
    }
}