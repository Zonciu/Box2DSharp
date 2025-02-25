using System;
using Box2DSharp;

namespace Testbed.Abstractions;

public class B2Random
{
    public static readonly B2Random Shared = new();

    public const int RandLimit = 32767;

    public const int RandSeed = 12345;

    // Global seed for simple random number generator. This is reset
    // for each sample.
    public uint Seed = RandSeed;

    public void Reset()
    {
        Seed = RandSeed;
    }

    // Simple random number generator. Using this instead of rand()
    // for cross platform determinism.
    public int RandomInt()
    {
        // XorShift32 algorithm
        var x = Seed;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        Seed = x;

        // Map the 32-bit value to the range 0 to RAND_LIMIT
        return (int)(x % (RandLimit + 1));
    }

    // Random integer in range [lo, hi]
    public float RandomInt(int lo, int hi)
    {
        return lo + RandomInt() % (hi - lo + 1);
    }

    // Random number in range [-1,1]
    public float RandomFloat()
    {
        float r = RandomInt() & RandLimit;
        r /= RandLimit;
        r = 2.0f * r - 1.0f;
        return r;
    }

    // Random floating point number in range [lo, hi]
    public float RandomFloat(float lo, float hi)
    {
        float r = RandomInt() & RandLimit;
        r /= RandLimit;
        r = (hi - lo) * r + lo;
        return r;
    }

    // Random vector with coordinates in range [lo, hi]
    public Vec2 RandomVec2(float lo, float hi)
    {
        Vec2 v;
        v.X = RandomFloat(lo, hi);
        v.Y = RandomFloat(lo, hi);
        return v;
    }

    public Polygon RandomPolygon(float extent)
    {
        Span<Vec2> points = stackalloc Vec2[Core.MaxPolygonVertices];
        var count = 3 + RandomInt() % 6;
        for (var i = 0; i < count; ++i)
        {
            points[i] = RandomVec2(-extent, extent);
        }

        var hull = HullFunc.ComputeHull(points, count);
        if (hull.Count > 0)
        {
            return Geometry.MakePolygon(hull, 0.0f);
        }

        return Geometry.MakeSquare(extent);
    }
}