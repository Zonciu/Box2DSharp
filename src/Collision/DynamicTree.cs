using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace Box2DSharp
{
    /// <summary>
    /// The dynamic tree structure. This should be considered private data.
    /// It is placed here for performance reasons.
    /// </summary>
    public class DynamicTree
    {
        /// <summary>
        /// The tree nodes
        /// </summary>
        public TreeNode[] Nodes;

        /// <summary>
        /// The root index
        /// </summary>
        public int Root;

        /// <summary>
        /// The number of nodes
        /// </summary>
        public int NodeCount;

        /// <summary>
        /// The allocated node space
        /// </summary>
        public int NodeCapacity;

        /// <summary>
        /// Node free list
        /// </summary>
        public int FreeList;

        /// <summary>
        /// Number of proxies created
        /// </summary>
        public int ProxyCount;

        /// <summary>
        /// Leaf indices for rebuild
        /// </summary>
        public int[] LeafIndices;

        /// <summary>
        /// Leaf bounding boxes for rebuild
        /// </summary>
        public AABB[] LeafBoxes;

        /// <summary>
        /// Leaf bounding box centers for rebuild
        /// </summary>
        public Vec2[] LeafCenters;

        /// <summary>
        /// Bins for sorting during rebuild
        /// </summary>
        public int[] BinIndices;

        /// <summary>
        /// Allocated space for rebuilding
        /// </summary>
        public int RebuildCapacity;

        private const int TreeStackSize = 1024;

        // TODO_ERIN
        // - try incrementally sorting internal nodes by height for better cache efficiency during depth first traversal.

        public DynamicTree()
        {
            Root = Core.NullIndex;

            NodeCapacity = 16;
            NodeCount = 0;
            Nodes = B2ArrayPool<TreeNode>.Shared.Rent(NodeCapacity);

            // Build a linked list for the free list.
            for (var i = 0; i < NodeCapacity - 1; ++i)
            {
                Nodes[i].Next = i + 1;
                Nodes[i].Height = -1;
            }

            Nodes[NodeCapacity - 1].Next = Core.NullIndex;
            Nodes[NodeCapacity - 1].Height = -1;
            FreeList = 0;

            ProxyCount = 0;

            LeafIndices = null!;
            LeafBoxes = null!;
            LeafCenters = null!;
            BinIndices = null!;
            RebuildCapacity = 0;
        }

        public void Destroy()
        {
            // b2Free(tree.nodes, tree.nodeCapacity * sizeof(b2TreeNode));
            // b2Free(tree.leafIndices, tree.rebuildCapacity * sizeof(int));
            // b2Free(tree.leafBoxes, tree.rebuildCapacity * sizeof(b2AABB));
            // b2Free(tree.leafCenters, tree.rebuildCapacity * sizeof(b2Vec2));
            // b2Free(tree.binIndices, tree.rebuildCapacity * sizeof(int));
            //
            // memset(tree, 0, sizeof(b2DynamicTree));
            B2ArrayPool<TreeNode>.Shared.Return(Nodes, true);
            B2ArrayPool<int>.Shared.Return(LeafIndices);
            B2ArrayPool<AABB>.Shared.Return(LeafBoxes);
            B2ArrayPool<Vec2>.Shared.Return(LeafCenters);
            B2ArrayPool<int>.Shared.Return(BinIndices);
            Nodes = null!;
            LeafIndices = null!;
            LeafBoxes = null!;
            LeafCenters = null!;
            BinIndices = null!;
        }

        // Allocate a node from the pool. Grow the pool if necessary.
        public int AllocateNode()
        {
            // Expand the node pool as needed.
            if (FreeList == Core.NullIndex)
            {
                Debug.Assert(NodeCount == NodeCapacity);

                // The free list is empty. Rebuild a bigger pool.
                var oldNodes = Nodes;
                var oldCapacity = NodeCapacity;
                NodeCapacity += oldCapacity >> 1;
                Nodes = B2ArrayPool<TreeNode>.Shared.Rent(NodeCapacity);
                Debug.Assert(oldNodes != null);
                oldNodes.AsSpan()[..NodeCount].CopyTo(Nodes);
                B2ArrayPool<TreeNode>.Shared.Return(oldNodes, true);

                // Build a linked list for the free list. The parent pointer becomes the "next" pointer.
                // todo avoid building freelist?
                for (var i = NodeCount; i < NodeCapacity - 1; ++i)
                {
                    Nodes[i].Next = i + 1;
                    Nodes[i].Height = -1;
                }

                Nodes[NodeCapacity - 1].Next = Core.NullIndex;
                Nodes[NodeCapacity - 1].Height = -1;
                FreeList = NodeCount;
            }

            // Peel a node off the free list.
            var nodeIndex = FreeList;
            ref var node = ref Nodes[nodeIndex];
            FreeList = node.Next;
            node.Init();
            ++NodeCount;
            return nodeIndex;
        }

        // Return a node to the pool.
        public void FreeNode(int nodeId)
        {
            Debug.Assert(0 <= nodeId && nodeId < NodeCapacity);
            Debug.Assert(0 < NodeCount);
            Nodes[nodeId].Next = FreeList;
            Nodes[nodeId].Height = -1;
            FreeList = nodeId;
            --NodeCount;
        }

        // Greedy algorithm for sibling selection using the SAH
        // We have three nodes A-(B,C) and want to add a leaf D, there are three choices.
        // 1: make a new parent for A and D : E-(A-(B,C), D)
        // 2: associate D with B
        //   a: B is a leaf : A-(E-(B,D), C)
        //   b: B is an internal node: A-(B{D},C)
        // 3: associate D with C
        //   a: C is a leaf : A-(B, E-(C,D))
        //   b: C is an internal node: A-(B, C{D})
        // All of these have a clear cost except when B or C is an internal node. Hence we need to be greedy.

        // The cost for cases 1, 2a, and 3a can be computed using the sibling cost formula.
        // cost of sibling H = area(union(H, D)) + increased are of ancestors

        // Suppose B (or C) is an internal node, then the lowest cost would be one of two cases:
        // case1: D becomes a sibling of B
        // case2: D becomes a descendant of B along with a new internal node of area(D).
        public int FindBestSibling(AABB boxD)
        {
            Vec2 centerD = B2Math.AABB_Center(boxD);
            float areaD = boxD.Perimeter;

            var nodeSpan = Nodes;
            int rootIndex = Root;

            AABB rootBox = nodeSpan[rootIndex].AABB;

            // Area of current node
            float areaBase = rootBox.Perimeter;

            // Area of inflated node
            float directCost = B2Math.AABB_Union(rootBox, boxD).Perimeter;
            float inheritedCost = 0.0f;

            int bestSibling = rootIndex;
            float bestCost = directCost;

            // Descend the tree from root, following a single greedy path.
            int index = rootIndex;
            while (nodeSpan[index].Height > 0)
            {
                int child1 = nodeSpan[index].Child1;
                int child2 = nodeSpan[index].Child2;

                // Cost of creating a new parent for this node and the new leaf
                float cost = directCost + inheritedCost;

                // Sometimes there are multiple identical costs within tolerance.
                // This breaks the ties using the centroid distance.
                if (cost < bestCost)
                {
                    bestSibling = index;
                    bestCost = cost;
                }

                // Inheritance cost seen by children
                inheritedCost += directCost - areaBase;

                bool leaf1 = nodeSpan[child1].Height == 0;
                bool leaf2 = nodeSpan[child2].Height == 0;

                // Cost of descending into child 1
                float lowerCost1 = float.MaxValue;
                AABB box1 = nodeSpan[child1].AABB;
                float directCost1 = B2Math.AABB_Union(box1, boxD).Perimeter;
                float area1 = 0.0f;
                if (leaf1)
                {
                    // Child 1 is a leaf
                    // Cost of creating new node and increasing area of node P
                    float cost1 = directCost1 + inheritedCost;

                    // Need this here due to while condition above
                    if (cost1 < bestCost)
                    {
                        bestSibling = child1;
                        bestCost = cost1;
                    }
                }
                else
                {
                    // Child 1 is an internal node
                    area1 = box1.Perimeter;

                    // Lower bound cost of inserting under child 1.
                    lowerCost1 = inheritedCost + directCost1 + Math.Min(areaD - area1, 0.0f);
                }

                // Cost of descending into child 2
                float lowerCost2 = float.MaxValue;
                AABB box2 = nodeSpan[child2].AABB;
                float directCost2 = B2Math.AABB_Union(box2, boxD).Perimeter;
                float area2 = 0.0f;
                if (leaf2)
                {
                    // Child 2 is a leaf
                    // Cost of creating new node and increasing area of node P
                    float cost2 = directCost2 + inheritedCost;

                    // Need this here due to while condition above
                    if (cost2 < bestCost)
                    {
                        bestSibling = child2;
                        bestCost = cost2;
                    }
                }
                else
                {
                    // Child 2 is an internal node
                    area2 = box2.Perimeter;

                    // Lower bound cost of inserting under child 2. This is not the cost
                    // of child 2, it is the best we can hope for under child 2.
                    lowerCost2 = inheritedCost + directCost2 + Math.Min(areaD - area2, 0.0f);
                }

                if (leaf1 && leaf2)
                {
                    break;
                }

                // Can the cost possibly be decreased?
                if (bestCost <= lowerCost1 && bestCost <= lowerCost2)
                {
                    break;
                }

                if (lowerCost1 == lowerCost2 && leaf1 == false)
                {
                    Debug.Assert(lowerCost1 < float.MaxValue);
                    Debug.Assert(lowerCost2 < float.MaxValue);

                    // No clear choice based on lower bound surface area. This can happen when both
                    // children fully contain D. Fall back to node distance.
                    Vec2 d1 = B2Math.Sub(B2Math.AABB_Center(box1), centerD);
                    Vec2 d2 = B2Math.Sub(B2Math.AABB_Center(box2), centerD);
                    lowerCost1 = B2Math.LengthSquared(d1);
                    lowerCost2 = B2Math.LengthSquared(d2);
                }

                // Descend
                if (lowerCost1 < lowerCost2 && leaf1 == false)
                {
                    index = child1;
                    areaBase = area1;
                    directCost = directCost1;
                }
                else
                {
                    index = child2;
                    areaBase = area2;
                    directCost = directCost2;
                }

                Debug.Assert(nodeSpan[index].Height > 0);
            }

            return bestSibling;
        }

        private enum RotateType
        {
            RotateNone,

            RotateBf,

            RotateBg,

            RotateCd,

            RotateCe
        }

        // Perform a left or right rotation if node A is imbalanced.
        // Returns the new root index.
        public void RotateNodes(int iA)
        {
            Debug.Assert(iA != Core.NullIndex);

            Span<TreeNode> nodeSpan = Nodes;

            ref var A = ref nodeSpan[iA];
            if (A.Height < 2)
            {
                return;
            }

            int iB = A.Child1;
            int iC = A.Child2;
            Debug.Assert(0 <= iB && iB < NodeCapacity);
            Debug.Assert(0 <= iC && iC < NodeCapacity);

            ref var B = ref nodeSpan[iB];
            ref var C = ref nodeSpan[iC];

            if (B.Height == 0)
            {
                // B is a leaf and C is internal
                Debug.Assert(C.Height > 0);

                int iF = C.Child1;
                int iG = C.Child2;
                ref var F = ref nodeSpan[iF];
                ref var G = ref nodeSpan[iG];
                Debug.Assert(0 <= iF && iF < NodeCapacity);
                Debug.Assert(0 <= iG && iG < NodeCapacity);

                // Base cost
                float costBase = C.AABB.Perimeter;

                // Cost of swapping B and F
                AABB aabbBG = B2Math.AABB_Union(B.AABB, G.AABB);
                float costBF = aabbBG.Perimeter;

                // Cost of swapping B and G
                AABB aabbBF = B2Math.AABB_Union(B.AABB, F.AABB);
                float costBG = aabbBF.Perimeter;

                if (costBase < costBF && costBase < costBG)
                {
                    // Rotation does not improve cost
                    return;
                }

                if (costBF < costBG)
                {
                    // Swap B and F
                    A.Child1 = iF;
                    C.Child1 = iB;

                    B.Parent = iC;
                    F.Parent = iA;

                    C.AABB = aabbBG;

                    C.Height = (short)(1 + Math.Max(B.Height, G.Height));
                    A.Height = (short)(1 + Math.Max(C.Height, F.Height));
                    C.CategoryBits = B.CategoryBits | G.CategoryBits;
                    A.CategoryBits = C.CategoryBits | F.CategoryBits;
                    C.Enlarged = B.Enlarged || G.Enlarged;
                    A.Enlarged = C.Enlarged || F.Enlarged;
                }
                else
                {
                    // Swap B and G
                    A.Child1 = iG;
                    C.Child2 = iB;

                    B.Parent = iC;
                    G.Parent = iA;

                    C.AABB = aabbBF;

                    C.Height = (short)(1 + Math.Max(B.Height, F.Height));
                    A.Height = (short)(1 + Math.Max(C.Height, G.Height));
                    C.CategoryBits = B.CategoryBits | F.CategoryBits;
                    A.CategoryBits = C.CategoryBits | G.CategoryBits;
                    C.Enlarged = B.Enlarged || F.Enlarged;
                    A.Enlarged = C.Enlarged || G.Enlarged;
                }
            }
            else if (C.Height == 0)
            {
                // C is a leaf and B is internal
                Debug.Assert(B.Height > 0);

                int iD = B.Child1;
                int iE = B.Child2;
                ref var D = ref nodeSpan[iD];
                ref var E = ref nodeSpan[iE];
                Debug.Assert(0 <= iD && iD < NodeCapacity);
                Debug.Assert(0 <= iE && iE < NodeCapacity);

                // Base cost
                float costBase = B.AABB.Perimeter;

                // Cost of swapping C and D
                AABB aabbCE = B2Math.AABB_Union(C.AABB, E.AABB);
                float costCD = aabbCE.Perimeter;

                // Cost of swapping C and E
                AABB aabbCD = B2Math.AABB_Union(C.AABB, D.AABB);
                float costCE = aabbCD.Perimeter;

                if (costBase < costCD && costBase < costCE)
                {
                    // Rotation does not improve cost
                    return;
                }

                if (costCD < costCE)
                {
                    // Swap C and D
                    A.Child2 = iD;
                    B.Child1 = iC;

                    C.Parent = iB;
                    D.Parent = iA;

                    B.AABB = aabbCE;

                    B.Height = (short)(1 + Math.Max(C.Height, E.Height));
                    A.Height = (short)(1 + Math.Max(B.Height, D.Height));
                    B.CategoryBits = C.CategoryBits | E.CategoryBits;
                    A.CategoryBits = B.CategoryBits | D.CategoryBits;
                    B.Enlarged = C.Enlarged || E.Enlarged;
                    A.Enlarged = B.Enlarged || D.Enlarged;
                }
                else
                {
                    // Swap C and E
                    A.Child2 = iE;
                    B.Child2 = iC;

                    C.Parent = iB;
                    E.Parent = iA;

                    B.AABB = aabbCD;
                    B.Height = (short)(1 + Math.Max(C.Height, D.Height));
                    A.Height = (short)(1 + Math.Max(B.Height, E.Height));
                    B.CategoryBits = C.CategoryBits | D.CategoryBits;
                    A.CategoryBits = B.CategoryBits | E.CategoryBits;
                    B.Enlarged = C.Enlarged || D.Enlarged;
                    A.Enlarged = B.Enlarged || E.Enlarged;
                }
            }
            else
            {
                int iD = B.Child1;
                int iE = B.Child2;
                int iF = C.Child1;
                int iG = C.Child2;

                ref var D = ref nodeSpan[iD];
                ref var E = ref nodeSpan[iE];
                ref var F = ref nodeSpan[iF];
                ref var G = ref nodeSpan[iG];

                Debug.Assert(0 <= iD && iD < NodeCapacity);
                Debug.Assert(0 <= iE && iE < NodeCapacity);
                Debug.Assert(0 <= iF && iF < NodeCapacity);
                Debug.Assert(0 <= iG && iG < NodeCapacity);

                // Base cost
                float areaB = B.AABB.Perimeter;
                float areaC = C.AABB.Perimeter;
                float costBase = areaB + areaC;
                RotateType bestRotation = RotateType.RotateNone;
                float bestCost = costBase;

                // Cost of swapping B and F
                AABB aabbBG = B2Math.AABB_Union(B.AABB, G.AABB);
                float costBF = areaB + aabbBG.Perimeter;
                if (costBF < bestCost)
                {
                    bestRotation = RotateType.RotateBf;
                    bestCost = costBF;
                }

                // Cost of swapping B and G
                AABB aabbBF = B2Math.AABB_Union(B.AABB, F.AABB);
                float costBG = areaB + aabbBF.Perimeter;
                if (costBG < bestCost)
                {
                    bestRotation = RotateType.RotateBg;
                    bestCost = costBG;
                }

                // Cost of swapping C and D
                AABB aabbCE = B2Math.AABB_Union(C.AABB, E.AABB);
                float costCD = areaC + aabbCE.Perimeter;
                if (costCD < bestCost)
                {
                    bestRotation = RotateType.RotateCd;
                    bestCost = costCD;
                }

                // Cost of swapping C and E
                AABB aabbCD = B2Math.AABB_Union(C.AABB, D.AABB);
                float costCE = areaC + aabbCD.Perimeter;
                if (costCE < bestCost)
                {
                    bestRotation = RotateType.RotateCe;

                    // bestCost = costCE;
                }

                switch (bestRotation)
                {
                case RotateType.RotateNone:
                    break;

                case RotateType.RotateBf:
                    A.Child1 = iF;
                    C.Child1 = iB;

                    B.Parent = iC;
                    F.Parent = iA;

                    C.AABB = aabbBG;
                    C.Height = (short)(1 + Math.Max(B.Height, G.Height));
                    A.Height = (short)(1 + Math.Max(C.Height, F.Height));
                    C.CategoryBits = B.CategoryBits | G.CategoryBits;
                    A.CategoryBits = C.CategoryBits | F.CategoryBits;
                    C.Enlarged = B.Enlarged || G.Enlarged;
                    A.Enlarged = C.Enlarged || F.Enlarged;
                    break;

                case RotateType.RotateBg:
                    A.Child1 = iG;
                    C.Child2 = iB;

                    B.Parent = iC;
                    G.Parent = iA;

                    C.AABB = aabbBF;
                    C.Height = (short)(1 + Math.Max(B.Height, F.Height));
                    A.Height = (short)(1 + Math.Max(C.Height, G.Height));
                    C.CategoryBits = B.CategoryBits | F.CategoryBits;
                    A.CategoryBits = C.CategoryBits | G.CategoryBits;
                    C.Enlarged = B.Enlarged || F.Enlarged;
                    A.Enlarged = C.Enlarged || G.Enlarged;
                    break;

                case RotateType.RotateCd:
                    A.Child2 = iD;
                    B.Child1 = iC;

                    C.Parent = iB;
                    D.Parent = iA;

                    B.AABB = aabbCE;
                    B.Height = (short)(1 + Math.Max(C.Height, E.Height));
                    A.Height = (short)(1 + Math.Max(B.Height, D.Height));
                    B.CategoryBits = C.CategoryBits | E.CategoryBits;
                    A.CategoryBits = B.CategoryBits | D.CategoryBits;
                    B.Enlarged = C.Enlarged || E.Enlarged;
                    A.Enlarged = B.Enlarged || D.Enlarged;
                    break;

                case RotateType.RotateCe:
                    A.Child2 = iE;
                    B.Child2 = iC;

                    C.Parent = iB;
                    E.Parent = iA;

                    B.AABB = aabbCD;
                    B.Height = (short)(1 + Math.Max(C.Height, D.Height));
                    A.Height = (short)(1 + Math.Max(B.Height, E.Height));
                    B.CategoryBits = C.CategoryBits | D.CategoryBits;
                    A.CategoryBits = B.CategoryBits | E.CategoryBits;
                    B.Enlarged = C.Enlarged || D.Enlarged;
                    A.Enlarged = B.Enlarged || E.Enlarged;
                    break;

                default:
                    throw new InvalidEnumArgumentException(nameof(bestRotation), (int)bestRotation, typeof(RotateType));
                }
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="leaf"></param>
        /// <param name="shouldRotate"></param>
        public void InsertLeaf(int leaf, bool shouldRotate)
        {
            if (Root == Core.NullIndex)
            {
                Root = leaf;
                Nodes[Root].Parent = Core.NullIndex;
                return;
            }

            // Stage 1: find the best sibling for this node
            AABB leafAABB = Nodes[leaf].AABB;
            int sibling = FindBestSibling(leafAABB);

            // Stage 2: create a new parent for the leaf and sibling
            int oldParent = Nodes[sibling].Parent;
            int newParent = AllocateNode();

            // warning: node pointer can change after allocation
            Span<TreeNode> nodeSpan = Nodes;
            nodeSpan[newParent].Parent = oldParent;
            nodeSpan[newParent].UserData = -1;
            nodeSpan[newParent].AABB = B2Math.AABB_Union(leafAABB, nodeSpan[sibling].AABB);
            nodeSpan[newParent].CategoryBits = nodeSpan[leaf].CategoryBits | nodeSpan[sibling].CategoryBits;
            nodeSpan[newParent].Height = (short)(nodeSpan[sibling].Height + 1);

            if (oldParent != Core.NullIndex)
            {
                // The sibling was not the root.
                if (nodeSpan[oldParent].Child1 == sibling)
                {
                    nodeSpan[oldParent].Child1 = newParent;
                }
                else
                {
                    nodeSpan[oldParent].Child2 = newParent;
                }

                nodeSpan[newParent].Child1 = sibling;
                nodeSpan[newParent].Child2 = leaf;
                nodeSpan[sibling].Parent = newParent;
                nodeSpan[leaf].Parent = newParent;
            }
            else
            {
                // The sibling was the root.
                nodeSpan[newParent].Child1 = sibling;
                nodeSpan[newParent].Child2 = leaf;
                nodeSpan[sibling].Parent = newParent;
                nodeSpan[leaf].Parent = newParent;
                Root = newParent;
            }

            // Stage 3: walk back up the tree fixing heights and AABBs
            int index = nodeSpan[leaf].Parent;
            while (index != Core.NullIndex)
            {
                int child1 = nodeSpan[index].Child1;
                int child2 = nodeSpan[index].Child2;

                Debug.Assert(child1 != Core.NullIndex);
                Debug.Assert(child2 != Core.NullIndex);

                nodeSpan[index].AABB = B2Math.AABB_Union(nodeSpan[child1].AABB, nodeSpan[child2].AABB);
                nodeSpan[index].CategoryBits = nodeSpan[child1].CategoryBits | nodeSpan[child2].CategoryBits;
                nodeSpan[index].Height = (short)(1 + Math.Max(nodeSpan[child1].Height, nodeSpan[child2].Height));
                nodeSpan[index].Enlarged = nodeSpan[child1].Enlarged || nodeSpan[child2].Enlarged;

                if (shouldRotate)
                {
                    RotateNodes(index);
                }

                index = nodeSpan[index].Parent;
            }
        }

        public void RemoveLeaf(int leaf)
        {
            if (leaf == Root)
            {
                Root = Core.NullIndex;
                return;
            }

            Span<TreeNode> nodeSpan = Nodes;

            int parent = nodeSpan[leaf].Parent;
            int grandParent = nodeSpan[parent].Parent;
            var sibling = nodeSpan[parent].Child1 == leaf ? nodeSpan[parent].Child2 : nodeSpan[parent].Child1;

            if (grandParent != Core.NullIndex)
            {
                // Destroy parent and connect sibling to grandParent.
                if (nodeSpan[grandParent].Child1 == parent)
                {
                    nodeSpan[grandParent].Child1 = sibling;
                }
                else
                {
                    nodeSpan[grandParent].Child2 = sibling;
                }

                nodeSpan[sibling].Parent = grandParent;
                FreeNode(parent);

                // Adjust ancestor bounds.
                int index = grandParent;
                while (index != Core.NullIndex)
                {
                    ref var node = ref nodeSpan[index];
                    ref var child1 = ref nodeSpan[node.Child1];
                    ref var child2 = ref nodeSpan[node.Child2];

                    // Fast union using SSE
                    //__m128 aabb1 = _mm_load_ps(&child1.aabb.lowerBound.x);
                    //__m128 aabb2 = _mm_load_ps(&child2.aabb.lowerBound.x);
                    //__m128 lower = _mm_min_ps(aabb1, aabb2);
                    //__m128 upper = _mm_max_ps(aabb1, aabb2);
                    //__m128 aabb = _mm_shuffle_ps(lower, upper, _MM_SHUFFLE(3, 2, 1, 0));
                    //_mm_store_ps(&node.aabb.lowerBound.x, aabb);

                    node.AABB = B2Math.AABB_Union(child1.AABB, child2.AABB);
                    node.CategoryBits = child1.CategoryBits | child2.CategoryBits;
                    node.Height = (short)(1 + Math.Max(child1.Height, child2.Height));

                    index = node.Parent;
                }
            }
            else
            {
                Root = sibling;
                Nodes[sibling].Parent = Core.NullIndex;
                FreeNode(parent);
            }
        }

        /// <summary>
        /// Create a proxy in the tree as a leaf node. We return the index of the node instead of a pointer so that we can grow
        /// the node pool.
        /// </summary>
        /// <param name="aabb"></param>
        /// <param name="categoryBits"></param>
        /// <param name="userData"></param>
        /// <returns></returns>
        public int CreateProxy(AABB aabb, ulong categoryBits, int userData)
        {
            Debug.Assert(-Core.Huge < aabb.LowerBound.X && aabb.LowerBound.X < Core.Huge);
            Debug.Assert(-Core.Huge < aabb.LowerBound.Y && aabb.LowerBound.Y < Core.Huge);
            Debug.Assert(-Core.Huge < aabb.UpperBound.X && aabb.UpperBound.X < Core.Huge);
            Debug.Assert(-Core.Huge < aabb.UpperBound.Y && aabb.UpperBound.Y < Core.Huge);

            int proxyId = AllocateNode();
            ref var node = ref Nodes[proxyId];

            node.AABB = aabb;
            node.UserData = userData;
            node.CategoryBits = categoryBits;
            node.Height = 0;

            bool shouldRotate = true;
            InsertLeaf(proxyId, shouldRotate);

            ProxyCount += 1;

            return proxyId;
        }

        public void DestroyProxy(int proxyId)
        {
            Debug.Assert(0 <= proxyId && proxyId < NodeCapacity);
            Debug.Assert(Nodes[proxyId].IsLeaf);

            RemoveLeaf(proxyId);
            FreeNode(proxyId);

            Debug.Assert(ProxyCount > 0);
            ProxyCount -= 1;
        }

        public int GetProxyCount()
        {
            return ProxyCount;
        }

        public void MoveProxy(int proxyId, AABB aabb)
        {
            Debug.Assert(aabb.IsValid);
            Debug.Assert(aabb.UpperBound.X - aabb.LowerBound.X < Core.Huge);
            Debug.Assert(aabb.UpperBound.Y - aabb.LowerBound.Y < Core.Huge);
            Debug.Assert(0 <= proxyId && proxyId < NodeCapacity);
            Debug.Assert(Nodes[proxyId].IsLeaf);

            RemoveLeaf(proxyId);

            Nodes[proxyId].AABB = aabb;

            bool shouldRotate = false;
            InsertLeaf(proxyId, shouldRotate);
        }

        public void EnlargeProxy(int proxyId, AABB aabb)
        {
            Span<TreeNode> nodeSpan = Nodes;

            Debug.Assert(aabb.IsValid);
            Debug.Assert(aabb.UpperBound.X - aabb.LowerBound.X < Core.Huge);
            Debug.Assert(aabb.UpperBound.Y - aabb.LowerBound.Y < Core.Huge);
            Debug.Assert(0 <= proxyId && proxyId < NodeCapacity);
            Debug.Assert(Nodes[proxyId].IsLeaf);

            // Caller must ensure this
            Debug.Assert(B2Math.AABB_Contains(nodeSpan[proxyId].AABB, aabb) == false);

            nodeSpan[proxyId].AABB = aabb;

            int parentIndex = nodeSpan[proxyId].Parent;
            while (parentIndex != Core.NullIndex)
            {
                bool changed = AABB.EnlargeAABB(ref nodeSpan[parentIndex].AABB, ref aabb);
                nodeSpan[parentIndex].Enlarged = true;
                parentIndex = nodeSpan[parentIndex].Parent;

                if (changed == false)
                {
                    break;
                }
            }

            while (parentIndex != Core.NullIndex)
            {
                if (nodeSpan[parentIndex].Enlarged == true)
                {
                    // early out because this ancestor was previously ascended and marked as enlarged
                    break;
                }

                nodeSpan[parentIndex].Enlarged = true;
                parentIndex = nodeSpan[parentIndex].Parent;
            }
        }

        public int GetHeight()
        {
            if (Root == Core.NullIndex)
            {
                return 0;
            }

            return Nodes[Root].Height;
        }

        public float GetAreaRatio()
        {
            if (Root == Core.NullIndex)
            {
                return 0.0f;
            }

            Span<TreeNode> nodeSpan = Nodes;
            ref var rootNode = ref nodeSpan[Root];
            float rootArea = rootNode.AABB.Perimeter;

            float totalArea = 0.0f;
            for (int i = 0; i < NodeCapacity; ++i)
            {
                ref var node = ref nodeSpan[i];
                if (node.Height < 0 || node.IsLeaf || i == Root)
                {
                    // Free node in pool
                    continue;
                }

                totalArea += node.AABB.Perimeter;
            }

            return totalArea / rootArea;
        }

        /// <summary>
        /// Compute the height of a sub-tree.
        /// </summary>
        /// <param name="nodeId"></param>
        /// <returns></returns>
        public int ComputeHeight(int nodeId)
        {
            Debug.Assert(0 <= nodeId && nodeId < NodeCapacity);
            ref var node = ref Nodes[nodeId];

            if (node.IsLeaf)
            {
                return 0;
            }

            var height1 = ComputeHeight(node.Child1);
            var height2 = ComputeHeight(node.Child2);
            return 1 + Math.Max(height1, height2);
        }

        /// <summary>
        /// Compute height from root
        /// </summary>
        /// <returns></returns>
        public int ComputeHeight() => ComputeHeight(Root);

        public void ValidateStructure(int index)
        {
            if (!Core.B2Validate)
            {
                return;
            }

            if (index == Core.NullIndex)
            {
                return;
            }

            if (index == Root)
            {
                Debug.Assert(Nodes[index].Parent == Core.NullIndex);
            }

            ref var node = ref Nodes[index];

            int child1 = node.Child1;
            int child2 = node.Child2;

            if (node.IsLeaf)
            {
                Debug.Assert(child1 == Core.NullIndex);
                Debug.Assert(child2 == Core.NullIndex);
                Debug.Assert(node.Height == 0);
                return;
            }

            Debug.Assert(0 <= child1 && child1 < NodeCapacity);
            Debug.Assert(0 <= child2 && child2 < NodeCapacity);

            Debug.Assert(Nodes[child1].Parent == index);
            Debug.Assert(Nodes[child2].Parent == index);

            if (Nodes[child1].Enlarged || Nodes[child2].Enlarged)
            {
                Debug.Assert(node.Enlarged == true);
            }

            ValidateStructure(child1);
            ValidateStructure(child2);
        }

        public void ValidateMetrics(int index)
        {
            if (!Core.B2Validate)
            {
                return;
            }

            if (index == Core.NullIndex)
            {
                return;
            }

            ref var node = ref Nodes[index];

            int child1 = node.Child1;
            int child2 = node.Child2;

            if (node.IsLeaf)
            {
                Debug.Assert(child1 == Core.NullIndex);
                Debug.Assert(child2 == Core.NullIndex);
                Debug.Assert(node.Height == 0);
                return;
            }

            Debug.Assert(0 <= child1 && child1 < NodeCapacity);
            Debug.Assert(0 <= child2 && child2 < NodeCapacity);

            int height1 = Nodes[child1].Height;
            int height2 = Nodes[child2].Height;
            int height;
            height = 1 + Math.Max(height1, height2);
            Debug.Assert(node.Height == height);

            // b2AABB aabb = b2Math.b2AABB_Union(tree.nodes[child1].aabb, tree.nodes[child2].aabb);

            Debug.Assert(B2Math.AABB_Contains(node.AABB, Nodes[child1].AABB));
            Debug.Assert(B2Math.AABB_Contains(node.AABB, Nodes[child2].AABB));

            // Debug.Assert(aabb.lowerBound.x == node.aabb.lowerBound.x);
            // Debug.Assert(aabb.lowerBound.y == node.aabb.lowerBound.y);
            // Debug.Assert(aabb.upperBound.x == node.aabb.upperBound.x);
            // Debug.Assert(aabb.upperBound.y == node.aabb.upperBound.y);

            var categoryBits = Nodes[child1].CategoryBits | Nodes[child2].CategoryBits;
            Debug.Assert(node.CategoryBits == categoryBits);

            ValidateMetrics(child1);
            ValidateMetrics(child2);
        }

        public void Validate()

        {
            if (!Core.B2Validate)
            {
                return;
            }

            if (Root == Core.NullIndex)
            {
                return;
            }

            ValidateStructure(Root);
            ValidateMetrics(Root);

            int freeCount = 0;
            int freeIndex = FreeList;
            while (freeIndex != Core.NullIndex)
            {
                Debug.Assert(0 <= freeIndex && freeIndex < NodeCapacity);
                freeIndex = Nodes[freeIndex].Next;
                ++freeCount;
            }

            int height = GetHeight();
            int computedHeight = ComputeHeight();
            Debug.Assert(height == computedHeight);

            Debug.Assert(NodeCount + freeCount == NodeCapacity);
        }

        public int GetMaxBalance()
        {
            int maxBalance = 0;
            var nodeSpan = Nodes.AsSpan();
            for (int i = 0; i < NodeCapacity; ++i)
            {
                ref var node = ref nodeSpan[i];
                if (node.Height <= 1)
                {
                    continue;
                }

                Debug.Assert(node.IsLeaf == false);

                int child1 = node.Child1;
                int child2 = node.Child2;
                int balance = Math.Abs(Nodes[child2].Height - Nodes[child1].Height);
                maxBalance = Math.Max(maxBalance, balance);
            }

            return maxBalance;
        }

        public void RebuildBottomUp()
        {
            var freeNodes = B2ArrayPool<int>.Shared.Rent(NodeCount);
            var count = 0;
            var treeSpan = Nodes.AsSpan();
            var freeSpan = freeNodes.AsSpan();

            // Build array of leaves. Free the rest.
            for (var i = 0; i < NodeCapacity; ++i)
            {
                if (treeSpan[i].Height < 0)
                {
                    // free node in pool
                    continue;
                }

                if (treeSpan[i].IsLeaf)
                {
                    treeSpan[i].Parent = Core.NullIndex;
                    freeSpan[count] = i;
                    ++count;
                }
                else
                {
                    FreeNode(i);
                }
            }

            while (count > 1)
            {
                float minCost = float.MaxValue;
                var iMin = -1;
                var jMin = -1;
                for (int i = 0; i < count; ++i)
                {
                    AABB aabbi = treeSpan[freeSpan[i]].AABB;

                    for (int j = i + 1; j < count; ++j)
                    {
                        AABB aabbj = treeSpan[freeSpan[j]].AABB;
                        AABB b = B2Math.AABB_Union(aabbi, aabbj);
                        float cost = b.Perimeter;
                        if (cost < minCost)
                        {
                            iMin = i;
                            jMin = j;
                            minCost = cost;
                        }
                    }
                }

                int index1 = freeSpan[iMin];
                int index2 = freeSpan[jMin];
                ref var child1 = ref treeSpan[index1];
                ref var child2 = ref treeSpan[index2];

                int parentIndex = AllocateNode();
                ref var parent = ref treeSpan[parentIndex];
                parent.Child1 = index1;
                parent.Child2 = index2;
                parent.AABB = B2Math.AABB_Union(child1.AABB, child2.AABB);
                parent.CategoryBits = child1.CategoryBits | child2.CategoryBits;
                parent.Height = (short)(1 + Math.Max(child1.Height, child2.Height));
                parent.Parent = Core.NullIndex;

                child1.Parent = parentIndex;
                child2.Parent = parentIndex;

                freeSpan[jMin] = freeSpan[count - 1];
                freeSpan[iMin] = parentIndex;
                --count;
            }

            Root = freeSpan[0];
            B2ArrayPool<int>.Shared.Return(freeNodes, true);
            Validate();
        }

        public void ShiftOrigin(Vec2 newOrigin)
        {
            // shift all AABBs
            var nodeSpan = Nodes.AsSpan();
            for (int i = 0; i < NodeCapacity; ++i)
            {
                ref var n = ref nodeSpan[i];
                n.AABB.LowerBound.X -= newOrigin.X;
                n.AABB.LowerBound.Y -= newOrigin.Y;
                n.AABB.UpperBound.X -= newOrigin.X;
                n.AABB.UpperBound.Y -= newOrigin.Y;
            }
        }

        public int GetByteCount()
        {
            return 0;
        }

        /// <summary>
        /// 查询AABB
        /// </summary>
        /// <param name="aabb"></param>
        /// <param name="maskBits"></param>
        /// <param name="callbackFcn"></param>
        /// <param name="context"></param>
        /// <typeparam name="T"></typeparam>
        public void Query(in AABB aabb, ulong maskBits, TreeQueryCallbackFcn callbackFcn, object context)
        {
            Span<int> stack = stackalloc int[TreeStackSize];
            int stackCount = 0;
            stack[stackCount++] = Root; // 从根节点遍历判断节点是否与输入的AABB有重叠
            var nodeSpan = Nodes.AsSpan();
            while (stackCount > 0)
            {
                // 弹栈取得节点
                int nodeId = stack[--stackCount];
                if (nodeId == Core.NullIndex)
                {
                    continue;
                }

                ref readonly var node = ref nodeSpan[nodeId];

                if (AABB.Overlaps(node.AABB, aabb) && (node.CategoryBits & maskBits) != 0)
                {
                    if (node.IsLeaf)
                    {
                        // 节点是叶子，执行回调
                        // callback to user code with proxy id
                        bool proceed = callbackFcn(nodeId, node.UserData, context);
                        if (proceed == false)
                        {
                            return;
                        }
                    }
                    else
                    {
                        // 节点不是叶子，则还有子节点，把子节点压入栈中
                        if (stackCount < TreeStackSize - 1)
                        {
                            stack[stackCount++] = node.Child1;
                            stack[stackCount++] = node.Child2;
                        }

                        Debug.Assert(stackCount < TreeStackSize - 1);
                    }
                }
            }
        }

        public void RayCast(ref RayCastInput rayCastInput, ulong maskBits, TreeRayCastCallbackFcn callbackFcn, object context)
        {
            Vec2 p1 = rayCastInput.Origin;
            Vec2 d = rayCastInput.Translation;

            Vec2 r = d.Normalize;

            // v is perpendicular to the segment.
            Vec2 v = B2Math.CrossSV(1.0f, r);
            Vec2 abs_v = B2Math.Abs(v);

            // Separating axis for segment (Gino, p80).
            // |dot(v, p1 - c)| > dot(|v|, h)

            float maxFraction = rayCastInput.MaxFraction;

            Vec2 p2 = B2Math.MulAdd(p1, maxFraction, d);

            // Build a bounding box for the segment.
            AABB segmentAABB = new(B2Math.Min(p1, p2), B2Math.Max(p1, p2));

            Span<int> stack = stackalloc int[TreeStackSize];
            int stackCount = 0;
            stack[stackCount++] = Root;
            var nodeSpan = Nodes.AsSpan();
            ref RayCastInput subInput = ref rayCastInput;

            while (stackCount > 0)
            {
                int nodeId = stack[--stackCount];
                if (nodeId == Core.NullIndex)
                {
                    continue;
                }

                ref var node = ref nodeSpan[nodeId];
                if (AABB.Overlaps(node.AABB, segmentAABB) == false || (node.CategoryBits & maskBits) == 0)
                {
                    continue;
                }

                // Separating axis for segment (Gino, p80).
                // |dot(v, p1 - c)| > dot(|v|, h)
                // radius extension is added to the node in this case
                Vec2 c = B2Math.AABB_Center(node.AABB);
                Vec2 h = B2Math.AABB_Extents(node.AABB);
                float term1 = Math.Abs(B2Math.Dot(v, B2Math.Sub(p1, c)));
                float term2 = B2Math.Dot(abs_v, h);
                if (term2 < term1)
                {
                    continue;
                }

                if (node.IsLeaf)
                {
                    subInput.MaxFraction = maxFraction;

                    float value = callbackFcn(ref subInput, nodeId, node.UserData, context);

                    if (value == 0.0f)
                    {
                        // The client has terminated the ray cast.
                        return;
                    }

                    if (0.0f < value && value < maxFraction)
                    {
                        // Update segment bounding box.
                        maxFraction = value;
                        p2 = B2Math.MulAdd(p1, maxFraction, d);
                        segmentAABB.LowerBound = B2Math.Min(p1, p2);
                        segmentAABB.UpperBound = B2Math.Max(p1, p2);
                    }
                }
                else
                {
                    if (stackCount < TreeStackSize - 1)
                    {
                        // TODO_ERIN just put one node on the stack, continue on a child node
                        // TODO_ERIN test ordering children by nearest to ray origin
                        stack[stackCount++] = node.Child1;
                        stack[stackCount++] = node.Child2;
                    }

                    Debug.Assert(stackCount < TreeStackSize - 1);
                }
            }
        }

        public void ShapeCast(ref ShapeCastInput input, ulong maskBits, TreeShapeCastCallbackFcn callback, object context)
        {
            if (input.Count == 0)
            {
                return;
            }

            AABB originAABB = new(input.Points[0], input.Points[0]);
            for (int i = 1; i < input.Count; ++i)
            {
                originAABB.LowerBound = B2Math.Min(originAABB.LowerBound, input.Points[i]);
                originAABB.UpperBound = B2Math.Max(originAABB.UpperBound, input.Points[i]);
            }

            Vec2 radius = new(input.Radius, input.Radius);

            originAABB.LowerBound = B2Math.Sub(originAABB.LowerBound, radius);
            originAABB.UpperBound = B2Math.Add(originAABB.UpperBound, radius);

            Vec2 p1 = B2Math.AABB_Center(originAABB);
            Vec2 extension = B2Math.AABB_Extents(originAABB);

            // v is perpendicular to the segment.
            Vec2 r = input.Translation;
            Vec2 v = B2Math.CrossSV(1.0f, r);
            Vec2 abs_v = B2Math.Abs(v);

            // Separating axis for segment (Gino, p80).
            // |dot(v, p1 - c)| > dot(|v|, h)

            float maxFraction = input.MaxFraction;

            // Build total box for the shape cast
            Vec2 t = B2Math.MulSV(maxFraction, input.Translation);
            AABB totalAABB = new(B2Math.Min(originAABB.LowerBound, B2Math.Add(originAABB.LowerBound, t)), B2Math.Max(originAABB.UpperBound, B2Math.Add(originAABB.UpperBound, t)));

            ref ShapeCastInput subInput = ref input;

            Span<int> stack = stackalloc int[TreeStackSize];
            int stackCount = 0;
            stack[stackCount++] = Root;
            var nodeSpan = Nodes.AsSpan();
            while (stackCount > 0)
            {
                int nodeId = stack[--stackCount];
                if (nodeId == Core.NullIndex)
                {
                    continue;
                }

                ref var node = ref nodeSpan[nodeId];
                if (AABB.Overlaps(node.AABB, totalAABB) == false || (node.CategoryBits & maskBits) == 0)
                {
                    continue;
                }

                // Separating axis for segment (Gino, p80).
                // |dot(v, p1 - c)| > dot(|v|, h)
                // radius extension is added to the node in this case
                Vec2 c = B2Math.AABB_Center(node.AABB);
                Vec2 h = B2Math.Add(B2Math.AABB_Extents(node.AABB), extension);
                float term1 = Math.Abs(B2Math.Dot(v, B2Math.Sub(p1, c)));
                float term2 = B2Math.Dot(abs_v, h);
                if (term2 < term1)
                {
                    continue;
                }

                if (node.IsLeaf)
                {
                    subInput.MaxFraction = maxFraction;

                    float value = callback(ref subInput, nodeId, node.UserData, context);

                    if (value == 0.0f)
                    {
                        // The client has terminated the ray cast.
                        return;
                    }

                    if (0.0f < value && value < maxFraction)
                    {
                        // Update segment bounding box.
                        maxFraction = value;
                        t = B2Math.MulSV(maxFraction, input.Translation);
                        totalAABB.LowerBound = B2Math.Min(originAABB.LowerBound, B2Math.Add(originAABB.LowerBound, t));
                        totalAABB.UpperBound = B2Math.Max(originAABB.UpperBound, B2Math.Add(originAABB.UpperBound, t));
                    }
                }
                else
                {
                    if (stackCount < TreeStackSize - 1)
                    {
                        // TODO_ERIN just put one node on the stack, continue on a child node
                        // TODO_ERIN test ordering children by nearest to ray origin
                        stack[stackCount++] = node.Child1;
                        stack[stackCount++] = node.Child2;
                    }

                    Debug.Assert(stackCount < TreeStackSize - 1);
                }
            }
        }

        // Median split == 0, Surface area heuristic == 1

        /// <summary>
        /// Median split heuristic
        /// </summary>
        /// <param name="indices"></param>
        /// <param name="centers"></param>
        /// <param name="count"></param>
        /// <returns></returns>
        public static int PartitionMid(Span<int> indices, Span<Vec2> centers, int count)
        {
            // Handle trivial case
            if (count <= 2)
            {
                return count / 2;
            }

            // todo SIMD?
            Vec2 lowerBound = centers[0];
            Vec2 upperBound = centers[0];

            for (int i = 1; i < count; ++i)
            {
                lowerBound = B2Math.Min(lowerBound, centers[i]);
                upperBound = B2Math.Max(upperBound, centers[i]);
            }

            Vec2 d = B2Math.Sub(upperBound, lowerBound);
            Vec2 c = new(0.5f * (lowerBound.X + upperBound.X), 0.5f * (lowerBound.Y + upperBound.Y));

            // Partition longest axis using the Hoare partition scheme
            // https://en.wikipedia.org/wiki/Quicksort
            // https://nicholasvadivelu.com/2021/01/11/array-partition/
            int i1 = 0, i2 = count;
            if (d.X > d.Y)
            {
                var pivot = c.X;

                while (i1 < i2)
                {
                    while (i1 < i2 && centers[i1].X < pivot)
                    {
                        i1 += 1;
                    }

                    while (i1 < i2 && centers[i2 - 1].X >= pivot)
                    {
                        i2 -= 1;
                    }

                    if (i1 < i2)
                    {
                        // Swap indices
                        (indices[i1], indices[i2 - 1]) = (indices[i2 - 1], indices[i1]);

                        // Swap centers
                        (centers[i1], centers[i2 - 1]) = (centers[i2 - 1], centers[i1]);

                        i1 += 1;
                        i2 -= 1;
                    }
                }
            }
            else
            {
                var pivot = c.Y;

                while (i1 < i2)
                {
                    while (i1 < i2 && centers[i1].Y < pivot)
                    {
                        i1 += 1;
                    }

                    while (i1 < i2 && centers[i2 - 1].Y >= pivot)
                    {
                        i2 -= 1;
                    }

                    if (i1 < i2)
                    {
                        // Swap indices
                        (indices[i1], indices[i2 - 1]) = (indices[i2 - 1], indices[i1]);

                        // Swap centers
                        (centers[i1], centers[i2 - 1]) = (centers[i2 - 1], centers[i1]);

                        i1 += 1;
                        i2 -= 1;
                    }
                }
            }

            Debug.Assert(i1 == i2);

            if (i1 > 0 && i1 < count)
            {
                return i1;
            }
            else
            {
                return count / 2;
            }
        }

        // Temporary data used to track the rebuild of a tree node
        private struct RebuildItem
        {
            public int NodeIndex;

            public int ChildCount;

            /// <summary>
            /// Leaf indices
            /// </summary>
            public int StartIndex;

            public int SplitIndex;

            public int EndIndex;
        }

        // Returns root node index
        public int BuildTree(int leafCount)
        {
            var nodeSpan = Nodes.AsSpan();
            var leafIndicesSpan = LeafIndices.AsSpan();
            var leafCentersSpan = LeafCenters.AsSpan();
            if (leafCount == 1)
            {
                nodeSpan[leafIndicesSpan[0]].Parent = Core.NullIndex;
                return leafIndicesSpan[0];
            }

            Span<RebuildItem> stack = stackalloc RebuildItem[TreeStackSize];
            int top = 0;

            stack[0].NodeIndex = AllocateNode();
            stack[0].ChildCount = -1;
            stack[0].StartIndex = 0;
            stack[0].EndIndex = leafCount;

            stack[0].SplitIndex = PartitionMid(leafIndicesSpan, leafCentersSpan, leafCount);

            while (true)
            {
                ref var item = ref stack[top];

                item.ChildCount += 1;

                if (item.ChildCount == 2)
                {
                    // This internal node has both children established

                    if (top == 0)
                    {
                        // all done
                        break;
                    }

                    ref var parentItem = ref stack[top - 1];
                    ref var parentNode = ref nodeSpan[parentItem.NodeIndex];

                    if (parentItem.ChildCount == 0)
                    {
                        Debug.Assert(parentNode.Child1 == Core.NullIndex);
                        parentNode.Child1 = item.NodeIndex;
                    }
                    else
                    {
                        Debug.Assert(parentItem.ChildCount == 1);
                        Debug.Assert(parentNode.Child2 == Core.NullIndex);
                        parentNode.Child2 = item.NodeIndex;
                    }

                    ref var node = ref nodeSpan[item.NodeIndex];

                    Debug.Assert(node.Parent == Core.NullIndex);
                    node.Parent = parentItem.NodeIndex;

                    Debug.Assert(node.Child1 != Core.NullIndex);
                    Debug.Assert(node.Child2 != Core.NullIndex);
                    ref var child1 = ref nodeSpan[node.Child1];
                    ref var child2 = ref nodeSpan[node.Child2];

                    node.AABB = B2Math.AABB_Union(child1.AABB, child2.AABB);
                    node.Height = (short)(1 + Math.Max(child1.Height, child2.Height));
                    node.CategoryBits = child1.CategoryBits | child2.CategoryBits;

                    // Pop stack
                    top -= 1;
                }
                else
                {
                    int startIndex, endIndex;
                    if (item.ChildCount == 0)
                    {
                        startIndex = item.StartIndex;
                        endIndex = item.SplitIndex;
                    }
                    else
                    {
                        Debug.Assert(item.ChildCount == 1);
                        startIndex = item.SplitIndex;
                        endIndex = item.EndIndex;
                    }

                    int count = endIndex - startIndex;

                    if (count == 1)
                    {
                        int childIndex = leafIndicesSpan[startIndex];
                        ref var node = ref nodeSpan[item.NodeIndex];

                        if (item.ChildCount == 0)
                        {
                            Debug.Assert(node.Child1 == Core.NullIndex);
                            node.Child1 = childIndex;
                        }
                        else
                        {
                            Debug.Assert(item.ChildCount == 1);
                            Debug.Assert(node.Child2 == Core.NullIndex);
                            node.Child2 = childIndex;
                        }

                        ref var childNode = ref nodeSpan[childIndex];
                        Debug.Assert(childNode.Parent == Core.NullIndex);
                        childNode.Parent = item.NodeIndex;
                    }
                    else
                    {
                        Debug.Assert(count > 0);
                        Debug.Assert(top < TreeStackSize);

                        top += 1;
                        ref var newItem = ref stack[top];
                        newItem.NodeIndex = AllocateNode();
                        newItem.ChildCount = -1;
                        newItem.StartIndex = startIndex;
                        newItem.EndIndex = endIndex;

                        newItem.SplitIndex = PartitionMid(leafIndicesSpan[startIndex..], leafCentersSpan[startIndex..], count);

                        newItem.SplitIndex += startIndex;
                    }
                }
            }

            ref var rootNode = ref nodeSpan[stack[0].NodeIndex];
            Debug.Assert(rootNode.Parent == Core.NullIndex);
            Debug.Assert(rootNode.Child1 != Core.NullIndex);
            Debug.Assert(rootNode.Child2 != Core.NullIndex);

            ref var c1 = ref nodeSpan[rootNode.Child1];
            ref var c2 = ref nodeSpan[rootNode.Child2];

            rootNode.AABB = B2Math.AABB_Union(c1.AABB, c2.AABB);
            rootNode.Height = (short)(1 + Math.Max(c1.Height, c2.Height));
            rootNode.CategoryBits = c1.CategoryBits | c2.CategoryBits;

            return stack[0].NodeIndex;
        }

        // Not safe to access tree during this operation because it may grow
        public int Rebuild(bool fullBuild)
        {
            if (ProxyCount == 0)
            {
                return 0;
            }

            // Ensure capacity for rebuild space
            if (ProxyCount > RebuildCapacity)
            {
                int newCapacity = ProxyCount + ProxyCount / 2;
                if (LeafIndices != null!)
                {
                    B2ArrayPool<int>.Shared.Return(LeafIndices);
                }

                LeafIndices = B2ArrayPool<int>.Shared.Rent(newCapacity);

                if (LeafCenters != null!)
                {
                    B2ArrayPool<Vec2>.Shared.Return(LeafCenters);
                }

                LeafCenters = B2ArrayPool<Vec2>.Shared.Rent(newCapacity);

                RebuildCapacity = newCapacity;
            }

            int leafCount = 0;
            Span<int> stack = stackalloc int[TreeStackSize];
            int stackCount = 0;

            int nodeIndex = Root;
            var nodeSpan = Nodes.AsSpan();
            ref var node = ref nodeSpan[nodeIndex];

            // These are the nodes that get sorted to rebuild the tree.
            // I'm using indices because the node pool may grow during the build.
            var leafIndicesSpan = LeafIndices.AsSpan();
            var leafCentersSpan = LeafCenters.AsSpan();

            // Gather all proxy nodes that have grown and all internal nodes that haven't grown. Both are
            // considered leaves in the tree rebuild.
            // Free all internal nodes that have grown.
            // todo use a node growth metric instead of simply enlarged to reduce rebuild size and frequency
            // this should be weighed against b2_aabbMargin
            while (true)
            {
                if (node.Height == 0 || (node.Enlarged == false && fullBuild == false))
                {
                    leafIndicesSpan[leafCount] = nodeIndex;

                    leafCentersSpan[leafCount] = B2Math.AABB_Center(node.AABB);

                    leafCount += 1;

                    // Detach
                    node.Parent = Core.NullIndex;
                }
                else
                {
                    int doomedNodeIndex = nodeIndex;

                    // Handle children
                    nodeIndex = node.Child1;

                    Debug.Assert(stackCount < TreeStackSize);
                    if (stackCount < TreeStackSize)
                    {
                        stack[stackCount++] = node.Child2;
                    }

                    node = ref nodeSpan[nodeIndex];

                    // Remove doomed node
                    FreeNode(doomedNodeIndex);

                    continue;
                }

                if (stackCount == 0)
                {
                    break;
                }

                nodeIndex = stack[--stackCount];
                node = ref nodeSpan[nodeIndex];
            }

            if (Core.B2Validate)
            {
                int capacity = NodeCapacity;
                for (int i = 0; i < capacity; ++i)
                {
                    if (nodeSpan[i].Height >= 0)
                    {
                        Debug.Assert(nodeSpan[i].Enlarged == false);
                    }
                }
            }

            Debug.Assert(leafCount <= ProxyCount);

            Root = BuildTree(leafCount);

            Validate();

            return leafCount;
        }

        /// Get proxy user data
        /// @return the proxy user data or 0 if the id is invalid
        public int GetUserData(int proxyId)
        {
            return Nodes[proxyId].UserData;
        }

        /// Get the AABB of a proxy
        public AABB GetAABB(int proxyId)
        {
            return Nodes[proxyId].AABB;
        }

        /// <summary>
        /// A node in the dynamic tree. This is private data placed here for performance reasons.
        /// </summary>
        [StructLayout(LayoutKind.Explicit, Size = 48)]
        public struct TreeNode
        {
            /// <summary>
            /// The node bounding box
            /// </summary>
            [FieldOffset(0)]
            public AABB AABB; // 16

            /// <summary>
            /// Category bits for collision filtering
            /// </summary>
            [FieldOffset(16)]
            public ulong CategoryBits; // 8

            /// <summary>
            /// The node parent index
            /// </summary>
            [FieldOffset(24)]
            public int Parent; // union with 'next' field

            /// <summary>
            /// The node freelist next index
            /// </summary>
            [FieldOffset(24)]
            public int Next;

            /// <summary>
            /// Child 1 index
            /// </summary>
            [FieldOffset(28)]
            public int Child1; // 4

            /// <summary>
            /// Child 2 index
            /// </summary>
            [FieldOffset(32)]
            public int Child2; // 4

            /// <summary>
            /// User data
            /// todo could be union with child index
            /// </summary>
            [FieldOffset(36)]
            public int UserData; // 4

            /// <summary>
            /// Leaf = 0, free node = -1
            /// </summary>
            [FieldOffset(40)]
            public short Height; // 2

            /// <summary>
            /// Has the AABB been enlarged?
            /// </summary>
            [FieldOffset(42)]
            public bool Enlarged; // 1

            [FieldOffset(43)]
            public int Pad1;

            [FieldOffset(47)]
            public byte Pad2;

            public bool IsLeaf => Height == 0;

            public void Init()
            {
                AABB = new AABB(Vec2.Zero, Vec2.Zero);
                CategoryBits = 0;
                Parent = Core.NullIndex;
                Child1 = Core.NullIndex;
                Child2 = Core.NullIndex;
                UserData = -1;
                Height = -2;
                Enlarged = false;
            }

            public override string ToString()
            {
                return $"H:{Height}/P:{Parent}/N:{Next}/C1:{Child1}/C2:{Child2}";
            }
        }
    }
}