using System.Numerics;

namespace Box2DSharp
{
    public static class BitTool
    {
        /// <summary>
        /// 从低位开始统计0的个数(32位)
        /// </summary>
        /// <param name="block"></param>
        /// <returns></returns>
        public static int CTZ32(uint block)
        {
            return BitOperations.TrailingZeroCount(block);
        }

        /// <summary>
        /// This function doesn't need to be fast, so using the Ivy Bridge fallback.
        /// 从高位开始统计0的个数(32位)
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        public static int CLZ32(uint value)
        {
            return BitOperations.LeadingZeroCount(value);
        }

        /// <summary>
        /// 从低位开始统计0的个数(64位)
        /// </summary>
        /// <param name="block"></param>
        /// <returns></returns>
        public static int CTZ64(ulong block)
        {
            return BitOperations.TrailingZeroCount(block);
        }

        public static bool IsPowerOf2(int x)
        {
            return (x & (x - 1)) == 0;
        }

        public static int BoundingPowerOf2(int x)
        {
            if (x <= 1)
            {
                return 1;
            }

            return 32 - CLZ32((uint)x - 1);
        }

        public static int RoundUpPowerOf2(int x)
        {
            if (x <= 1)
            {
                return 1;
            }

            return 1 << (32 - CLZ32((uint)x - 1));
        }
    }
}