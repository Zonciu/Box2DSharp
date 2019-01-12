namespace Box2DSharp.Common
{
    public struct FixedArray2<T>
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

        public T this[int index]
        {
            get => Values[index];
            set => Values[index] = value;
        }

        public void Initialize() => Values = new T[2];

        public static FixedArray2<T> Create() => new FixedArray2<T>() {Values = new T[2]};
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

        public T this[int index]
        {
            get => Values[index];
            set => Values[index] = value;
        }

        public void Initialize() => Values = new T[3];

        public static FixedArray3<T> Create() => new FixedArray3<T>() {Values = new T[3]};
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

        public T this[int index]
        {
            get => Values[index];
            set => Values[index] = value;
        }

        public void Initialize() => Values = new T[4];

        public static FixedArray4<T> Create() => new FixedArray4<T>() {Values = new T[4]};
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

        public T this[int index]
        {
            get => Values[index];
            set => Values[index] = value;
        }

        public void Initialize() => Values = new T[8];

        public static FixedArray8<T> Create() => new FixedArray8<T>() {Values = new T[8]};
    }
}