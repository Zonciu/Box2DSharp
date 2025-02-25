using Box2DSharp;
using FluentAssertions;
using Xunit;

namespace UnitTest;

public class CollisionTest
{
    [Fact]
    public int AABBTest()
    {
        AABB a = new();
        a.LowerBound = (-1.0f, -1.0f);
        a.UpperBound = (-2.0f, -2.0f);

        a.IsValid.Should().BeFalse();

        a.UpperBound = (1.0f, 1.0f);
        a.IsValid.Should().BeTrue();

        AABB b = ((2.0f, 2.0f), (4.0f, 4.0f));
        AABB.Overlaps(a, b).Should().BeFalse();
        AABB.Contains(a, b).Should().BeFalse();

        Vec2 p1 = (-2.0f, 0.0f);
        Vec2 p2 = (2.0f, 0.0f);

        CastOutput output = a.RayCast(p1, p2);
        output.Hit.Should().BeTrue();
        output.Fraction.Should().BeInRange(0.1f, 0.9f);

        return 0;
    }
}