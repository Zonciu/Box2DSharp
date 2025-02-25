using System.Diagnostics;

namespace Box2DSharp
{
    public static class Guard
    {
        public static void CheckIndex<T>(this B2Array<T> a, int index)
        {
            Debug.Assert(0 <= index && index < a.Count);
        }

        public static void CheckId(this B2Array<Shape> array, int id)
        {
            Debug.Assert(0 <= id && id < array.Count && array[id].Id == id);
        }

        public static void CheckId(this B2Array<Body> array, int id)
        {
            Debug.Assert(0 <= id && id < array.Count && array[id].Id == id);
        }

        public static void CheckId(this B2Array<ChainShape> array, int id)
        {
            Debug.Assert(0 <= id && id < array.Count && array[id].Id == id);
        }

        public static void CheckIdAndRevision(this B2Array<Shape> array, int id, int rev)
        {
            Debug.Assert(0 <= id && id < array.Count && array[id].Id == id && array[id].Revision == rev);
        }

        public static void CheckIdAndRevision(this B2Array<ChainShape> array, int id, int rev)
        {
            Debug.Assert(0 <= id && id < array.Count && array[id].Id == id && array[id].Revision == rev);
        }
    }
}