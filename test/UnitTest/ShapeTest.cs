using System;
using Box2DSharp;
using FluentAssertions;
using Xunit;

namespace UnitTest;

public class ShapeTest
{
    static Capsule capsule = ((-1.0f, 0.0f), (1.0f, 0.0f), 1.0f);

    static Circle circle = ((1.0f, 0.0f), 1.0f);

    static Polygon box;

    static Segment segment = ((0.0f, 1.0f), (0.0f, -1.0f));

    public const int N = 4;

    public ShapeTest()
    {
        box = Geometry.MakeBox(1.0f, 1.0f);
    }

    [Fact]
    public int ShapeMassTest()
    {
        {
            MassData md = Geometry.ComputeCircleMass(circle, 1.0f);
            md.Mass.Should().BeApproximately(B2Math.Pi, B2Math.FloatEpsilon);
            md.Center.X.Should().Be(1.0f);
            md.Center.Y.Should().Be(0.0f);
            md.RotationalInertia.Should().BeApproximately(1.5f * B2Math.Pi, B2Math.FloatEpsilon);
        }

        {
            float radius = capsule.Radius;
            float length = B2Math.Distance(capsule.Center1, capsule.Center2);

            MassData md = Geometry.ComputeCapsuleMass(capsule, 1.0f);

            // Box that full contains capsule
            Polygon r = Geometry.MakeBox(radius, radius + 0.5f * length);
            MassData mdr = Geometry.ComputePolygonMass(r, 1.0f);

            // Approximate capsule using convex hull
            Vec2[] points = new Vec2[2 * N];
            float d = B2Math.Pi / (N - 1.0f);
            float angle = -0.5f * B2Math.Pi;
            for (int i = 0; i < N; ++i)
            {
                points[i].X = 1.0f + radius * MathF.Cos(angle);
                points[i].Y = radius * MathF.Sin(angle);
                angle += d;
            }

            angle = 0.5f * B2Math.Pi;
            for (int i = N; i < 2 * N; ++i)
            {
                points[i].X = -1.0f + radius * MathF.Cos(angle);
                points[i].Y = radius * MathF.Sin(angle);
                angle += d;
            }

            Hull hull = HullFunc.ComputeHull(points, 2 * N);
            Polygon ac = Geometry.MakePolygon(hull, 0.0f);
            MassData ma = Geometry.ComputePolygonMass(ac, 1.0f);
            ma.Mass.Should().BeLessThan(md.Mass);
            md.Mass.Should().BeLessThan(mdr.Mass);
            ma.RotationalInertia.Should().BeLessThan(md.RotationalInertia);

            md.RotationalInertia.Should().BeLessThan(mdr.RotationalInertia);
        }

        {
            MassData md = Geometry.ComputePolygonMass(box, 1.0f);
            md.Mass.Should().BeApproximately(4.0f, B2Math.FloatEpsilon);
            md.Center.X.Should().BeApproximately(0f, B2Math.FloatEpsilon);
            md.Center.Y.Should().BeApproximately(0f, B2Math.FloatEpsilon);
            md.RotationalInertia.Should().BeApproximately(8.0f / 3.0f, 2.0f * B2Math.FloatEpsilon);
        }

        return 0;
    }

    [Fact]
    public int ShapeAABBTest()
    {
        {
            AABB b = Geometry.ComputeCircleAABB(circle, Transform.Identity);
            b.LowerBound.X.Should().BeApproximately(0, B2Math.FloatEpsilon);
            b.LowerBound.Y.Should().BeApproximately(-1.0f, B2Math.FloatEpsilon);
            b.UpperBound.X.Should().BeApproximately(2.0f, B2Math.FloatEpsilon);
            b.UpperBound.Y.Should().BeApproximately(1.0f, B2Math.FloatEpsilon);
        }

        {
            AABB b = Geometry.ComputePolygonAABB(box, Transform.Identity);
            b.LowerBound.X.Should().BeApproximately(-1.0f, B2Math.FloatEpsilon);
            b.LowerBound.Y.Should().BeApproximately(-1.0f, B2Math.FloatEpsilon);
            b.UpperBound.X.Should().BeApproximately(1.0f, B2Math.FloatEpsilon);
            b.UpperBound.Y.Should().BeApproximately(1.0f, B2Math.FloatEpsilon);
        }

        {
            AABB b = Geometry.ComputeSegmentAABB(segment, Transform.Identity);
            b.LowerBound.X.Should().BeApproximately(0, B2Math.FloatEpsilon);
            b.LowerBound.Y.Should().BeApproximately(-1.0f, B2Math.FloatEpsilon);
            b.UpperBound.X.Should().BeApproximately(0, B2Math.FloatEpsilon);
            b.UpperBound.Y.Should().BeApproximately(1.0f, B2Math.FloatEpsilon);
        }

        return 0;
    }

    [Fact]
    public int PointInShapeTest()
    {
        Vec2 p1 = (0.5f, 0.5f);
        Vec2 p2 = (4.0f, -4.0f);

        {
            bool hit;
            hit = Geometry.PointInCircle(p1, circle);
            hit.Should().BeTrue();
            hit = Geometry.PointInCircle(p2, circle);
            hit.Should().BeFalse();
        }

        {
            bool hit;
            hit = Geometry.PointInPolygon(p1, box);
            hit.Should().BeTrue();
            hit = Geometry.PointInPolygon(p2, box);
            hit.Should().BeFalse();
        }

        return 0;
    }

    [Fact]
    public int RayCastShapeTest()
    {
        RayCastInput input = new RayCastInput((-4.0f, 0.0f), (8.0f, 0.0f), 1.0f);

        {
            CastOutput output = Geometry.RayCastCircle(input, circle);
            output.Hit.Should().BeTrue();
            output.Normal.X.Should().BeApproximately(-1f, B2Math.FloatEpsilon);
            output.Normal.Y.Should().BeApproximately(0f, B2Math.FloatEpsilon);
            output.Fraction.Should().BeApproximately(0.5f, B2Math.FloatEpsilon);
        }

        {
            CastOutput output = Geometry.RayCastPolygon(input, box);
            output.Hit.Should().BeTrue();
            output.Normal.X.Should().BeApproximately(-1f, B2Math.FloatEpsilon);
            output.Normal.Y.Should().BeApproximately(0f, B2Math.FloatEpsilon);
            output.Fraction.Should().BeApproximately(3.0f / 8.0f, B2Math.FloatEpsilon);
        }

        {
            CastOutput output = Geometry.RayCastSegment(input, segment, true);
            output.Hit.Should().BeTrue();
            output.Normal.X.Should().BeApproximately(-1f, B2Math.FloatEpsilon);
            output.Normal.Y.Should().BeApproximately(0f, B2Math.FloatEpsilon);
            output.Fraction.Should().BeApproximately(0.5f, B2Math.FloatEpsilon);
        }

        return 0;
    }
}