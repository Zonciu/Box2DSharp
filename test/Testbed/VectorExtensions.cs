using System.Numerics;
using Box2DSharp;

namespace Testbed;

public static class VectorExtensions
{
    public static Vector2 ToVector2(this Vec2 vector)
    {
        return new Vector2(vector.X, vector.Y);
    }

    public static Vector2 ToVector2(this OpenTK.Mathematics.Vector2 vector)
    {
        return new Vector2(vector.X, vector.Y);
    }

    public static void Set(ref this Vector2 vector, float x, float y)
    {
        vector.X = x;
        vector.Y = y;
    }
}