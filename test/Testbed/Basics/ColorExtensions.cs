using Box2DSharp.Common;
using OpenToolkit.Mathematics;

namespace Testbed.Basics
{
    public static class ColorExtensions
    {
        public static Color4 ToColor4(in this Color color)
        {
            return new Color4(color.R / 255f, color.G / 255f, color.B / 255f, color.A / 255f);
        }
    }
}