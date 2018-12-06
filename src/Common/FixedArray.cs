using System;

namespace Box2DSharp.Common
{
    public static class FixedArrayExtensions
    {
        public static ref T GetRef<T>(ref this FixedArray2<T> array, int index)
        {
            switch (index)
            {
            case 0:
                return ref array.Value0;
            case 1:
                return ref array.Value1;
            default:
                throw new IndexOutOfRangeException();
            }
        }

        public static ref T GetRef<T>(ref this FixedArray3<T> array, int index)
        {
            switch (index)
            {
            case 0:
                return ref array.Value0;
            case 1:
                return ref array.Value1;
            case 2:
                return ref array.Value2;
            default:
                throw new IndexOutOfRangeException();
            }
        }

        public static ref T GetRef<T>(ref this FixedArray4<T> array, int index)
        {
            switch (index)
            {
            case 0:
                return ref array.Value0;
            case 1:
                return ref array.Value1;
            case 2:
                return ref array.Value2;
            case 3:
                return ref array.Value3;
            default:
                throw new IndexOutOfRangeException();
            }
        }

        public static ref T GetRef<T>(ref this FixedArray8<T> array, int index)
        {
            switch (index)
            {
            case 0:
                return ref array.Value0;
            case 1:
                return ref array.Value1;
            case 2:
                return ref array.Value2;
            case 3:
                return ref array.Value3;
            case 4:
                return ref array.Value4;
            case 5:
                return ref array.Value5;
            case 6:
                return ref array.Value6;
            case 7:
                return ref array.Value7;
            default:
                throw new IndexOutOfRangeException();
            }
        }
    }

    public struct FixedArray2<T>
    {
        public T Value0;

        public T Value1;

        public T this[int index]
        {
            get
            {
                switch (index)
                {
                case 0:
                    return Value0;
                case 1:
                    return Value1;
                default:
                    throw new IndexOutOfRangeException();
                }
            }
            set
            {
                switch (index)
                {
                case 0:
                    Value0 = value;
                    break;
                case 1:
                    Value1 = value;
                    break;
                default:
                    throw new IndexOutOfRangeException();
                }
            }
        }
    }

    public struct FixedArray3<T>
    {
        public T Value0;

        public T Value1;

        public T Value2;

        public T this[int index]
        {
            get
            {
                switch (index)
                {
                case 0:
                    return Value0;
                case 1:
                    return Value1;
                case 2:
                    return Value2;
                default:
                    throw new IndexOutOfRangeException();
                }
            }
            set
            {
                switch (index)
                {
                case 0:
                    Value0 = value;
                    break;
                case 1:
                    Value1 = value;
                    break;
                case 2:
                    Value2 = value;
                    break;
                default:
                    throw new IndexOutOfRangeException();
                }
            }
        }
    }

    public struct FixedArray4<T>
    {
        public T Value0;

        public T Value1;

        public T Value2;

        public T Value3;

        public T this[int index]
        {
            get
            {
                switch (index)
                {
                case 0:
                    return Value0;
                case 1:
                    return Value1;
                case 2:
                    return Value2;
                case 3:
                    return Value3;
                default:
                    throw new IndexOutOfRangeException();
                }
            }
            set
            {
                switch (index)
                {
                case 0:
                    Value0 = value;
                    break;
                case 1:
                    Value1 = value;
                    break;
                case 2:
                    Value2 = value;
                    break;
                case 3:
                    Value3 = value;
                    break;
                default:
                    throw new IndexOutOfRangeException();
                }
            }
        }
    }

    public struct FixedArray8<T>
    {
        public T Value0;

        public T Value1;

        public T Value2;

        public T Value3;

        public T Value4;

        public T Value5;

        public T Value6;

        public T Value7;

        public T this[int index]
        {
            get
            {
                switch (index)
                {
                case 0:
                    return Value0;
                case 1:
                    return Value1;
                case 2:
                    return Value2;
                case 3:
                    return Value3;
                case 4:
                    return Value4;
                case 5:
                    return Value5;
                case 6:
                    return Value6;
                case 7:
                    return Value7;
                default:
                    throw new IndexOutOfRangeException();
                }
            }
            set
            {
                switch (index)
                {
                case 0:
                    Value0 = value;
                    break;
                case 1:
                    Value1 = value;
                    break;
                case 2:
                    Value2 = value;
                    break;
                case 3:
                    Value3 = value;
                    break;
                case 4:
                    Value4 = value;
                    break;
                case 5:
                    Value5 = value;
                    break;
                case 6:
                    Value6 = value;
                    break;
                case 7:
                    Value7 = value;
                    break;
                default:
                    throw new IndexOutOfRangeException();
                }
            }
        }
    }
}