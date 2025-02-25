using Box2DSharp;
using FluentAssertions;
using Xunit;

namespace UnitTest;

public class BitSetTest
{
    private const int Count = 169;

    [Fact]
    public int BitSet()
    {
        BitSet bitSet = new BitSet(Count);
        bitSet.SetBitCountAndClear(Count);

        bool[] values = new bool[Count];

        var i1 = 0;
        var i2 = 1;
        bitSet.SetBit(i1);
        values[i1] = true;

        while (i2 < Count)
        {
            bitSet.SetBit(i2);

            values[i2] = true;
            var next = i1 + i2;
            i1 = i2;
            i2 = next;
        }

        for (var i = 0; i < Count; ++i)
        {
            bool value = bitSet.GetBit(i);
            value.Should().Be(values[i]);
        }

        bitSet.Dispose();

        return 0;
    }
}