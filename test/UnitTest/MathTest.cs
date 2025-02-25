using System;
using System.Numerics;
using Box2DSharp;
using FluentAssertions;
using Xunit;

namespace UnitTest;

public class MathTest
{
    [Fact]
    public int TestMath()
    {
        for (float x = -10.0f * B2Math.Pi; x < 10.0f * B2Math.Pi; x += 0.01f)
        {
            Rot r = B2Math.MakeRot(x);
            float c = MathF.Cos(x);
            float s = MathF.Sin(x);

            // The cosine and sine approximations are accurate to about 0.1 degrees (0.002 radians) 
            //printf( "%g %g\n", r.c - c, r.s - s );
            r.C.Should().BeApproximately(c, 0.002f);
            r.S.Should().BeApproximately(s, 0.002f);

            float xn = B2Math.UnwindLargeAngle(x);
            float a = B2Math.Atan2(s, c);
            float diff = MathF.Abs(a - xn);

            // The two results can be off by 360 degrees (-pi and pi)
            if (diff > B2Math.Pi)
            {
                diff -= 2.0f * B2Math.Pi;
            }

            // The approximate atan2 is quite accurate
            diff.Should().BeInRange(-1e-5f, 1e-5f);
        }

        Vec2 zero = Vec2.Zero;
        Vec2 one = (1.0f, 1.0f);
        Vec2 two = (2.0f, 2.0f);

        Vec2 v = B2Math.Add(one, two);
        v.X.Should().Be(3.0f);
        v.Y.Should().Be(3.0f);

        v = B2Math.Sub(zero, two);
        v.X.Should().Be(-2.0f);
        v.Y.Should().Be(-2.0f);

        v = B2Math.Add(two, two);
        v.X.Should().NotBe(5.0f);
        v.Y.Should().NotBe(5.0f);

        Transform transform1 = ((-2.0f, 3.0f), B2Math.MakeRot(1.0f));
        Transform transform2 = ((1.0f, 0.0f), B2Math.MakeRot(-2.0f));

        Transform transform = B2Math.MulTransforms(transform2, transform1);

        v = B2Math.TransformPoint(transform2, B2Math.TransformPoint(transform1, two));

        Vec2 u = B2Math.TransformPoint(transform, two);
        u.X.Should().BeApproximately(v.X, 10 * B2Math.FloatEpsilon);
        u.Y.Should().BeApproximately(v.Y, 10 * B2Math.FloatEpsilon);

        v = B2Math.TransformPoint(transform1, two);
        v = B2Math.InvTransformPoint(transform1, v);

        v.X.Should().BeApproximately(two.X, 8 * B2Math.FloatEpsilon);
        v.Y.Should().BeApproximately(two.Y, 8 * B2Math.FloatEpsilon);

        return 0;
    }

    [Fact]
    public void BlendTest()
    {
        var testA = new Vector<float>([-1.47458851f, 0, 0, 0, 0, 0, 0, 0]);
        var testB = new Vector<float>([-20.2496319f, 0, 0, 0, 0, 0, 0, 0]);
        var testMask = new Vector<float>();
        var value = BlendW(testA, testB, testMask);
        value[0].Should().Be(-1.47458851f);

        var value2 = ContactSolver.BlendW(testA, testB, testMask);
        value2[0].Should().Be(-1.47458851f);

        static Vector<float> BlendW(Vector<float> a, Vector<float> b, Vector<float> mask)
        {
            Vector<float> r = new(
            [
                mask[0] != 0.0f ? b[0] : a[0],
                mask[1] != 0.0f ? b[1] : a[1],
                mask[2] != 0.0f ? b[2] : a[2],
                mask[3] != 0.0f ? b[3] : a[3],
                mask[4] != 0.0f ? b[4] : a[4],
                mask[5] != 0.0f ? b[5] : a[5],
                mask[6] != 0.0f ? b[6] : a[6],
                mask[7] != 0.0f ? b[7] : a[7]
            ]);
            return r;
        }
    }
}