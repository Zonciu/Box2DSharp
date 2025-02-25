using System;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace Box2DSharp
{
    public struct B2SetItem
    {
        public ulong Key;

        public uint Hash;
    }

    public class B2HashSet
    {
        public B2SetItem[] Items;

        public int Capacity;

        public int Count;

        public B2HashSet(int capacity)
        {
            // Capacity must be a power of 2
            Capacity = capacity > 16 ? BitTool.RoundUpPowerOf2(capacity) : 16;

            Count = 0;
            Items = B2ArrayPool<B2SetItem>.Shared.Rent(capacity);
            Array.Clear(Items, 0, Items.Length);
        }

        public void DestroySet()
        {
            B2ArrayPool<B2SetItem>.Shared.Return(Items);
            Items = null!;
            Count = 0;
            Capacity = 0;
        }

        public void ClearSet()
        {
            Count = 0;
            Array.Clear(Items, 0, Items.Length);
        }

        // I need a good hash because the keys are built from pairs of increasing integers.
        // A simple hash like hash = (integer1 XOR integer2) has many collisions.
        // https://lemire.Me/blog/2018/08/15/fast-strongly-universal-64-bit-hashing-everywhere/
        // https://preshing.Com/20130107/this-hash-set-is-faster-than-a-judy-array/
        // TODO_ERIN try: https://www.Jandrewrogers.Com/2019/02/12/fast-perfect-hashing/
        public static uint KeyHash(ulong key)
        {
            ulong h = key;
            h ^= h >> 33;
            h *= 0xff51afd7ed558ccdL;
            h ^= h >> 33;
            h *= 0xc4ceb9fe1a85ec53L;
            h ^= h >> 33;

            return (uint)h;
        }

        public int FindSlot(ulong key, uint hash)
        {
            int capacity = Capacity;
            int index = (int)(hash & (capacity - 1));
            var items = Items;
            while (items[index].Hash != 0 && items[index].Key != key)
            {
                index = (index + 1) & (capacity - 1);
            }

            return index;
        }

        public void AddKeyHaveCapacity(ulong key, uint hash)
        {
            int index = FindSlot(key, hash);
            var items = Items;
            Debug.Assert(items[index].Hash == 0);
            ref var item = ref items[index];
            item.Key = key;
            item.Hash = hash;
            Count += 1;
        }

        public void GrowTable()
        {
            var oldCount = Count;

            var oldCapacity = Capacity;
            var oldItems = Items;

            Count = 0;

            // Capacity must be a power of 2
            Capacity = 2 * oldCapacity;
            Items = B2ArrayPool<B2SetItem>.Shared.Rent(Capacity);
            Array.Clear(Items, 0, Items.Length);

            // Transfer items into new array
            for (uint i = 0; i < oldCapacity; ++i)
            {
                B2SetItem item = oldItems[i];
                if (item.Hash == 0)
                {
                    // this item was empty
                    continue;
                }

                AddKeyHaveCapacity(item.Key, item.Hash);
            }

            Debug.Assert(Count == oldCount);
            B2ArrayPool<B2SetItem>.Shared.Return(oldItems);
        }

        public bool ContainsKey(int key) => ContainsKey((ulong)key);

        public bool ContainsKey(ulong key)
        {
            // key of zero is a sentinel
            Debug.Assert(key != 0);
            uint hash = KeyHash(key);
            int index = FindSlot(key, hash);
            return Items[index].Key == key;
        }

        public int GetHashSetBytes()
        {
            return Capacity * Marshal.SizeOf<B2SetItem>();
        }

        public bool AddKey(int key) => AddKey((ulong)key);

        public bool AddKey(ulong key)
        {
            // key of zero is a sentinel
            Debug.Assert(key != 0);

            uint hash = KeyHash(key);
            Debug.Assert(hash != 0);

            int index = FindSlot(key, hash);
            if (Items[index].Hash != 0)
            {
                // Already in set
                Debug.Assert(Items[index].Hash == hash && Items[index].Key == key);
                return true;
            }

            if (2 * Count >= Capacity)
            {
                GrowTable();
            }

            AddKeyHaveCapacity(key, hash);
            return false;
        }

        public bool RemoveKey(int key) => RemoveKey((ulong)key);

        // See https://en.Wikipedia.Org/wiki/Open_addressing
        public bool RemoveKey(ulong key)
        {
            uint hash = KeyHash(key);
            int i = FindSlot(key, hash);
            var items = Items;
            ref var item = ref items[i];
            if (item.Hash == 0)
            {
                // Not in set
                return false;
            }

            // Mark item i as unoccupied
            item.Key = 0;
            item.Hash = 0;

            Debug.Assert(Count > 0);
            Count -= 1;

            // Attempt to fill item i
            int j = i;
            var capacity = Capacity;
            for (;;)
            {
                j = (j + 1) & (capacity - 1);
                if (items[j].Hash == 0)
                {
                    break;
                }

                // k is the first item for the hash of j
                int k = (int)(items[j].Hash & (capacity - 1));

                // determine if k lies cyclically in (i,j]
                // i <= j: | i..K..J |
                // i > j: |.K..J  i....| or |....J     i..K.|
                if (i <= j)
                {
                    if (i < k && k <= j)
                    {
                        continue;
                    }
                }
                else
                {
                    if (i < k || k <= j)
                    {
                        continue;
                    }
                }

                // Move j into i
                items[i] = items[j];

                // Mark item j as unoccupied
                ref var itemJ = ref items[j];
                itemJ.Key = 0;
                itemJ.Hash = 0;

                i = j;
            }

            return true;
        }
    }
}