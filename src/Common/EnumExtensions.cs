using System.Runtime.CompilerServices;

namespace Box2DSharp
{
    /// <summary>
    /// 
    /// </summary>
    public static class EnumExtensions
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsSet(this ContactSimFlags self, ContactSimFlags flag) => (self & flag) == flag;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsSet(this ContactFlags self, ContactFlags flag) => (self & flag) == flag;
    }
}