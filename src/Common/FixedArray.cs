using System;
using System.Diagnostics;

namespace Box2DSharp.Common
{
    public static class FixedArrayExtensions
    {
        public static ref T GetRef<T>(ref this FixedArray2<T> array, int index)
        {
            if (index > -1 && index < 2)
            {
                return ref array.Values[index];
            }

            throw new IndexOutOfRangeException($"invalid index {index}");
        }

        public static ref T GetRef<T>(ref this FixedArray3<T> array, int index)
        {
            if (index > -1 && index < 3)
            {
                return ref array.Values[index];
            }

            throw new IndexOutOfRangeException($"invalid index {index}");
        }

        public static ref T GetRef<T>(ref this FixedArray4<T> array, int index)
        {
            if (index > -1 && index < 4)
            {
                return ref array.Values[index];
            }

            throw new IndexOutOfRangeException($"invalid index {index}");
        }

        public static ref T GetRef<T>(ref this FixedArray8<T> array, int index)
        {
            if (index > -1 && index < 8)
            {
                return ref array.Values[index];
            }

            throw new IndexOutOfRangeException($"invalid index {index}");
        }
    }

    public struct FixedArray2<T>
    {
        public T[] Values;

        public static FixedArray2<T> Create()
        {
            return new FixedArray2<T>()
            {
                Values = new T[2]
            };
        }

        public T Value0
        {
            get => Values[0];
            set => Values[0] = value;
        }

        public T Value1
        {
            get => Values[1];
            set => Values[1] = value;
        }

        public T this[int index]
        {
            get
            {
                if (index > -1 && index < 2)
                {
                    return Values[index];
                }

                throw new IndexOutOfRangeException($"invalid index {index}");
            }
            set
            {
                if (index > -1 && index < 2)
                {
                    Values[index] = value;
                }
                else
                {
                    throw new IndexOutOfRangeException($"invalid index {index}");
                }
            }
        }
    }

    public struct FixedArray3<T>
    {
        public T[] Values;

        public T Value0
        {
            get => Values[0];
            set => Values[0] = value;
        }

        public T Value1
        {
            get => Values[1];
            set => Values[1] = value;
        }

        public T Value2
        {
            get => Values[2];
            set => Values[2] = value;
        }

        public static FixedArray3<T> Create()
        {
            return new FixedArray3<T>()
            {
                Values = new T[3]
            };
        }

        public T this[int index]
        {
            get
            {
                if (index > -1 && index < 3)
                {
                    return Values[index];
                }

                throw new IndexOutOfRangeException($"invalid index {index}");
            }
            set
            {
                if (index > -1 && index < 3)
                {
                    Values[index] = value;
                }
                else
                {
                    throw new IndexOutOfRangeException($"invalid index {index}");
                }
            }
        }
    }

    public struct FixedArray4<T>
    {
        public T[] Values;

        public T Value0
        {
            get => Values[0];
            set => Values[0] = value;
        }

        public T Value1
        {
            get => Values[1];
            set => Values[1] = value;
        }

        public T Value2
        {
            get => Values[2];
            set => Values[2] = value;
        }

        public T Value3
        {
            get => Values[3];
            set => Values[3] = value;
        }

        public static FixedArray4<T> Create()
        {
            return new FixedArray4<T>()
            {
                Values = new T[4]
            };
        }

        public T this[int index]
        {
            get
            {
                if (index > -1 && index < 4)
                {
                    return Values[index];
                }

                throw new IndexOutOfRangeException($"invalid index {index}");
            }
            set
            {
                if (index > -1 && index < 4)
                {
                    Values[index] = value;
                }
                else
                {
                    throw new IndexOutOfRangeException($"invalid index {index}");
                }
            }
        }
    }

    public struct FixedArray8<T>
    {
        public T[] Values;

        public T Value0
        {
            get => Values[0];
            set => Values[0] = value;
        }

        public T Value1
        {
            get => Values[1];
            set => Values[1] = value;
        }

        public T Value2
        {
            get => Values[2];
            set => Values[2] = value;
        }

        public T Value3
        {
            get => Values[3];
            set => Values[3] = value;
        }

        public T Value4
        {
            get => Values[4];
            set => Values[4] = value;
        }

        public T Value5
        {
            get => Values[5];
            set => Values[5] = value;
        }

        public T Value6
        {
            get => Values[6];
            set => Values[6] = value;
        }

        public T Value7
        {
            get => Values[7];
            set => Values[7] = value;
        }

        public static FixedArray8<T> Create()
        {
            return new FixedArray8<T>()
            {
                Values = new T[8]
            };
        }

        public T this[int index]
        {
            get
            {
                if (index > -1 && index < 8)
                {
                    return Values[index];
                }

                throw new IndexOutOfRangeException($"invalid index {index}");
            }
            set
            {
                if (index > -1 && index < 8)
                {
                    Values[index] = value;
                }
                else
                {
                    throw new IndexOutOfRangeException($"invalid index {index}");
                }
            }
        }
    }
}