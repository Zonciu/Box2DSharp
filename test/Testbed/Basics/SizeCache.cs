using System.Runtime.InteropServices;

namespace Testbed.Basics
{
    public static class SizeCache<T>
    {
        public static readonly int Size = Marshal.SizeOf<T>();
    }
}