using System;
using Box2DSharp;
using FluentAssertions;
using Xunit;

namespace UnitTest;

public class DistanceTest
{
    [Fact]
    public int SegmentDistanceTest()
    {
        Vec2 p1 = (-1.0f, -1.0f);
        Vec2 q1 = (-1.0f, 1.0f);
        Vec2 p2 = (2.0f, 0.0f);
        Vec2 q2 = (1.0f, 0.0f);

        var result = DistanceFunc.SegmentDistance(p1, q1, p2, q2);
        result.Fraction1.Should().BeApproximately(0.5f, B2Math.FloatEpsilon);
        result.Fraction2.Should().BeApproximately(1f, B2Math.FloatEpsilon);
        result.Closest1.X.Should().BeApproximately(-1f, B2Math.FloatEpsilon);
        result.Closest1.Y.Should().BeApproximately(0f, B2Math.FloatEpsilon);
        result.Closest2.X.Should().BeApproximately(1f, B2Math.FloatEpsilon);
        result.Closest2.Y.Should().BeApproximately(0f, B2Math.FloatEpsilon);
        result.DistanceSquared.Should().BeApproximately(4.0f, B2Math.FloatEpsilon);

        return 0;
    }

    [Fact]
    public int ShapeDistanceTest()
    {
        Vec2[] vas =
        [
            (-1.0f, -1.0f),
            (1.0f, -1.0f),
            (1.0f, 1.0f),
            (-1.0f, 1.0f)
        ];

        Vec2[] vbs =
        [
            (2.0f, -1.0f),
            (2.0f, 1.0f),
        ];

        DistanceInput input = new();
        input.ProxyA = DistanceFunc.MakeProxy(vas, vas.Length, 0.0f);
        input.ProxyB = DistanceFunc.MakeProxy(vbs, vbs.Length, 0.0f);
        input.TransformA = Transform.Identity;
        input.TransformB = Transform.Identity;
        input.UseRadii = false;

        DistanceCache cache = new();
        DistanceOutput output = DistanceFunc.ShapeDistance(ref cache, input, null, 0);
        output.Distance.Should().BeApproximately(1f, B2Math.FloatEpsilon);

        return 0;
    }

    [Fact]
    public int ShapeCastTest()
    {
        Vec2[] vas =
        [
            (-1.0f, -1.0f),
            (1.0f, -1.0f),
            (1.0f, 1.0f),
            (-1.0f, 1.0f)
        ];

        Vec2[] vbs =
        [
            (2.0f, -1.0f),
            (2.0f, 1.0f),
        ];

        ShapeCastPairInput input = new();
        input.ProxyA = DistanceFunc.MakeProxy(vas, vas.Length, 0.0f);
        input.ProxyB = DistanceFunc.MakeProxy(vbs, vbs.Length, 0.0f);
        input.TransformA = Transform.Identity;
        input.TransformB = Transform.Identity;
        input.TranslationB = (-2.0f, 0.0f);
        input.MaxFraction = 1.0f;

        var output = DistanceFunc.ShapeCast(input);

        output.Hit.Should().BeTrue();
        output.Fraction.Should().BeApproximately(0.5f, 0.005f);

        return 0;
    }

    [Fact]
    public int TimeOfImpactTest()
    {
        Vec2[] vas =
        [
            (-1.0f, -1.0f),
            (1.0f, -1.0f),
            (1.0f, 1.0f),
            (-1.0f, 1.0f)
        ];

        Vec2[] vbs =
        [
            (2.0f, -1.0f),
            (2.0f, 1.0f),
        ];

        TOIInput input = new();
        input.ProxyA = DistanceFunc.MakeProxy(vas, vas.Length, 0.0f);
        input.ProxyB = DistanceFunc.MakeProxy(vbs, vbs.Length, 0.0f);
        input.SweepA = new Sweep(Vec2.Zero, Vec2.Zero, Vec2.Zero, Rot.Identity, Rot.Identity);
        input.SweepB = new Sweep(Vec2.Zero, Vec2.Zero, (-2.0f, 0.0f), Rot.Identity, Rot.Identity);
        input.TMax = 1.0f;

        TOIOutput output = DistanceFunc.TimeOfImpact(input);
        output.State.Should().Be(TOIState.Hit);
        output.T.Should().BeApproximately(0.5f, 0.005f);
        return 0;
    }
}