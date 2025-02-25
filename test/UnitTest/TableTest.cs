using System.Collections.Generic;
using System.Diagnostics;
using Box2DSharp;
using FluentAssertions;
using Xunit;
using Xunit.Abstractions;

namespace UnitTest;

public class TableTest
{
    private readonly ITestOutputHelper output;

    public TableTest(ITestOutputHelper output)
    {
        this.output = output;
    }

    const int SET_SPAN = 317;

    private const int ITEM_COUNT = ((SET_SPAN * SET_SPAN - SET_SPAN) / 2);

    [Fact]
    public int TestTable()
    {
        int power = BitTool.BoundingPowerOf2(3008);
        power.Should().Be(12);

        int nextPowerOf2 = BitTool.RoundUpPowerOf2(3008);
        nextPowerOf2.Should().Be(1 << power);

        const int N = SET_SPAN;
        const int itemCount = ITEM_COUNT;
        bool[] removed = new bool[ITEM_COUNT];

        for (var iter = 0; iter < 1; ++iter)
        {
            B2HashSet set = new B2HashSet(16);

            // Fill set
            for (var i = 0; i < N; ++i)
            {
                for (var j = i + 1; j < N; ++j)
                {
                    var key = Core.ShapePairKey(i, j);
                    set.AddKey(key);
                }
            }

            set.Count.Should().Be(itemCount);

            // Remove a portion of the set
            int k = 0;
            int removeCount = 0;
            for (var i = 0; i < N; ++i)
            {
                for (var j = i + 1; j < N; ++j)
                {
                    if (j == i + 1)
                    {
                        var key = Core.ShapePairKey(i, j);
                        set.RemoveKey(key);
                        removed[k++] = true;
                        removeCount += 1;
                    }
                    else
                    {
                        removed[k++] = false;
                    }
                }
            }

            set.Count.Should().Be(itemCount - removeCount);

            // Test key search
            // ~5ns per search on an AMD 7950x
            var timer = Stopwatch.GetTimestamp();

            k = 0;
            for (var i = 0; i < N; ++i)
            {
                for (var j = i + 1; j < N; ++j)
                {
                    var key = Core.ShapePairKey(j, i);
                    if (!set.ContainsKey(key))
                    {
                        removed[k].Should().BeTrue();
                    }

                    k += 1;
                }
            }

            var ms = StopwatchHelper.GetElapsedTime(timer);

            output.WriteLine("set: count = {0}, b2ContainsKey = {1} ms, ave = {2} us", itemCount, ms, 1000.0f * ms / itemCount);

            // Remove all keys from set
            for (var i = 0; i < N; ++i)
            {
                for (var j = i + 1; j < N; ++j)
                {
                    var key = Core.ShapePairKey(i, j);
                    set.RemoveKey(key);
                }
            }

            set.Count.Should().Be(0);
        }

        return 0;
    }
}