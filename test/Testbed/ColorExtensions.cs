using Box2DSharp;
using OpenTK.Mathematics;

namespace Testbed
{
    public static class ColorExtensions
    {
        public static Color4 ToColor4(this B2HexColor color)
        {
            var val = (int)color;
            var r = (byte)((val >> 16) & 0xFF);
            var g = (byte)((val >> 8) & 0xFF);
            var b = (byte)(val & 0xFF);
            return new Color4(r, g, b, 255);
        }

        public static Color4 ToColor4(this B2HexColor color, float alpha)
        {
            var val = (int)color;
            var r = (byte)((val >> 16) & 0xFF);
            var g = (byte)((val >> 8) & 0xFF);
            var b = (byte)(val & 0xFF);
            return new Color4(r, g, b, (byte)(255 * alpha));
        }
    }
}