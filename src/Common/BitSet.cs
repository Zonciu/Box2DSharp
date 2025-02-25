using System;
using System.Diagnostics;

namespace Box2DSharp
{
    public class BitSet
    {
        public ulong[] Bits;

        public int BlockCapacity;

        public int BlockCount;

        public void SetBit(int bitIndex)
        {
            var blockIndex = bitIndex / 64;
            Debug.Assert(blockIndex < BlockCount);
            Bits[blockIndex] |= ((ulong)1 << (bitIndex % 64));
        }

        public void SetBitGrow(int bitIndex)
        {
            var blockIndex = bitIndex / 64;
            if (blockIndex >= BlockCount)
            {
                GrowBitSet(blockIndex + 1);
            }

            Bits[blockIndex] |= (ulong)1 << bitIndex % 64;
        }

        public void ClearBit(int bitIndex)
        {
            var blockIndex = bitIndex / 64;
            if (blockIndex >= BlockCount)
            {
                return;
            }

            Bits[blockIndex] &= ~((ulong)1 << bitIndex % 64);
        }

        public bool GetBit(int bitIndex)
        {
            var blockIndex = bitIndex / 64;
            if (blockIndex >= BlockCount)
            {
                return false;
            }

            return (Bits[blockIndex] & ((ulong)1 << bitIndex % 64)) != 0;
        }

        public int GetBitSetBytes()
        {
            return BlockCapacity * sizeof(ulong);
        }

        public BitSet(int bitCapacity)
        {
            BlockCapacity = (bitCapacity + sizeof(ulong) * 8 - 1) / (sizeof(ulong) * 8);
            BlockCount = 0;
            Bits = B2ArrayPool<ulong>.Shared.Rent(BlockCapacity);
            Bits.AsSpan().Clear();
        }

        public void Dispose()
        {
            B2ArrayPool<ulong>.Shared.Return(Bits);
            BlockCapacity = 0;
            BlockCount = 0;
            Bits = null!;
        }

        public void SetBitCountAndClear(int bitCount)
        {
            BlockCount = (bitCount + sizeof(ulong) * 8 - 1) / (sizeof(ulong) * 8);
            if (BlockCapacity < BlockCount)
            {
                BlockCapacity = bitCount + (bitCount >> 1);
                B2ArrayPool<ulong>.Shared.Resize(ref Bits, BlockCapacity);
            }

            Bits.AsSpan().Clear();
        }

        public void GrowBitSet(int blocks)
        {
            Debug.Assert(blocks > BlockCount);
            if (blocks > BlockCapacity)
            {
                BlockCapacity = blocks + blocks / 2;
                B2ArrayPool<ulong>.Shared.Resize(ref Bits, BlockCapacity);
                Debug.Assert(Bits != null);
            }

            BlockCount = blocks;
        }

        public void InPlaceUnion(BitSet setB)
        {
            Debug.Assert(BlockCount == setB.BlockCount);
            for (var i = 0; i < BlockCount; ++i)
            {
                Bits[i] |= setB.Bits[i];
            }
        }
    }
}