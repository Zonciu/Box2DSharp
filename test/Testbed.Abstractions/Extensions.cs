namespace Testbed.Abstractions
{
    public static class Extensions
    {
        public static T[] Fill<T>(this T[] array)
            where T : new()
        {
            for (var i = 0; i < array.Length; i++)
            {
                array[i] = new T();
            }

            return array;
        }
    }
}