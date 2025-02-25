using System;

namespace Box2DSharp
{
    public static class HashTool
    {
        public const uint HashInit = 5381;

        public static uint Hash(uint hash, ReadOnlySpan<byte> data)
        {
            var result = hash;
            foreach (var t in data)
            {
                result = (result << 5) + result + t;
            }

            return result;
        }
    }
}