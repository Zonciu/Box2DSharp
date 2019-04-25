using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DSharp.Common
{
    [StructLayout(LayoutKind.Sequential)]
    public struct FixedArray2<T>
        where T : unmanaged
    {
        public T Value0;

        public T Value1;

        public ref T this[int index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                if (index > -1 && index < 2)
                {
                    unsafe
                    {
                        fixed (T* ptr = &Value0)
                        {
                            return ref *(ptr + index);
                        }
                    }
                }

                throw new IndexOutOfRangeException($"{nameof(FixedArray2<T>)} index can't be {index}");
            }
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct FixedArray3<T>
        where T : unmanaged
    {
        public T Value0;

        public T Value1;

        public T Value2;

        public ref T this[int index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                if (index > -1 && index < 3)
                {
                    unsafe
                    {
                        fixed (T* ptr = &Value0)
                        {
                            return ref *(ptr + index);
                        }
                    }
                }

                throw new IndexOutOfRangeException($"{nameof(FixedArray3<T>)} index can't be {index}");
            }
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct FixedArray4<T>
        where T : unmanaged
    {
        public T Value0;

        public T Value1;

        public T Value2;

        public T Value3;

        public ref T this[int index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                if (index > -1 && index < 4)
                {
                    unsafe
                    {
                        fixed (T* ptr = &Value0)
                        {
                            return ref *(ptr + index);
                        }
                    }
                }

                throw new IndexOutOfRangeException($"{nameof(FixedArray4<T>)} index can't be {index}");
            }
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct FixedArray8<T>
        where T : unmanaged
    {
        public T Value0;

        public T Value1;

        public T Value2;

        public T Value3;

        public T Value4;

        public T Value5;

        public T Value6;

        public T Value7;

        public ref T this[int index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                if (index > -1 && index < 8)
                {
                    unsafe
                    {
                        fixed (T* ptr = &Value0)
                        {
                            return ref *(ptr + index);
                        }
                    }
                }

                throw new IndexOutOfRangeException($"{nameof(FixedArray8<T>)} index can't be {index}");
            }
        }
    }
}