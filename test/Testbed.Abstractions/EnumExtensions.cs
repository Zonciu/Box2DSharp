namespace Testbed.Abstractions
{
    public static class EnumExtensions
    {
        public static bool IsSet(this KeyModifiers self, KeyModifiers flag) => (self & flag) == flag;
    }
}