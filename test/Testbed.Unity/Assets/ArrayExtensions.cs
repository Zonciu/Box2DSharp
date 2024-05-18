namespace Box2DSharp.Testbed.Unity
{
    public static class ArrayExtensions
    {
        public static T[] Fill<T>(this T[] array)
            where T : new()
        {
            for (int i = 0; i < array.Length; i++)
            {
                array[i] = new T();
            }

            return array;
        }
    }
}