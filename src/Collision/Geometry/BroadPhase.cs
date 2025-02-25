using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;

namespace Box2DSharp
{
    /// <summary>
    /// The broad-phase is used for computing pairs and performing volume queries and ray casts.
    /// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
    /// It is up to the client to consume the new pairs and to track subsequent overlap.
    /// </summary>
    public class BroadPhase
    {
        public readonly DynamicTree[] Trees = new DynamicTree[(int)BodyType.BodyTypeCount];

        public int ProxyCount;

        /// <summary>
        /// The move set and array are used to track shapes that have moved significantly and need a pair query for new contacts. The array has a deterministic order.<br/>
        /// todo perhaps just a move set?<br/>
        /// todo implement a 32bit hash set for faster lookup<br/>
        /// todo moveSet can grow quite large on the first time step and remain large<br/>
        /// 移动集合，用于跟踪有明显移动的形状，需要给新接触点创建配对查询，具有确定的顺序。其内容是ProxyKey，包含了ShapeId和BodyType<br/>
        /// todo 尝试使用SortedSet来合并MoveSet和MoveArray
        /// </summary>
        public readonly B2HashSet MoveSet;

        public readonly B2Array<int> MoveArray;

        /// <summary>
        /// These are the results from the pair query and are used to create new contacts in deterministic order.<br/>
        /// todo these could be in the step context<br/>
        /// 配对查询的结果，用于按确定顺序创建新接触点
        /// </summary>
        public MoveResult[] MoveResults;

        public int MoveResultCount;

        //public MovePair[] MovePairs;

        //public int MovePairCapacity;

        /// <summary>
        /// _Atomic
        /// </summary>

        //public int MovePairIndex;

        // Tracks shape pairs that have a b2Contact
        // todo pairSet can grow quite large on the first time step and remain large
        public readonly B2HashSet PairSet;

        /// <summary>
        /// Store the proxy type in the lower 2 bits of the proxy key. This leaves 30 bits for the id.<br/>
        /// 获取刚体类型
        /// </summary>
        /// <param name="key"></param>
        /// <returns></returns>
        public static int ProxyType(int key)
        {
            return key & 3;
        }

        /// <summary>
        /// 获取代理Id
        /// </summary>
        /// <param name="key"></param>
        /// <returns></returns>
        public static int ProxyId(int key)
        {
            return key >> 2;
        }

        /// <summary>
        /// 代理Id和刚体类型合并到一个int的代理Key中。低2位是刚体类型，高30位保存Id
        /// </summary>
        /// <param name="id"></param>
        /// <param name="type"></param>
        /// <returns></returns>
        public static int ProxyKey(int id, BodyType type)
        {
            return (id << 2) | (int)type;
        }

        public BroadPhase()
        {
            Debug.Assert(BodyType.BodyTypeCount == (BodyType)3, "must be three body types");

            // if (s_file == NULL)
            //{
            //	s_file = fopen("pairs01.txt", "a");
            //	fprintf(s_file, "============\n\n");
            // }

            ProxyCount = 0;
            MoveSet = new(16);
            MoveArray = new B2Array<int>(16);
            MoveResults = null!;

            //MovePairs = null!;
            //MovePairCapacity = 0;
            //MovePairIndex = 0;
            PairSet = new(32);

            for (var i = 0; i < Core.BodyTypeCount; ++i)
            {
                Trees[i] = new DynamicTree();
            }
        }

        public void DestroyBroadPhase()
        {
            for (var i = 0; i < Core.BodyTypeCount; ++i)
            {
                Trees[i].Destroy();
            }

            MoveSet.DestroySet();
            MoveArray.Dispose();
            PairSet.ClearSet();
        }

        public void UnBufferMove(int proxyKey)
        {
            bool found = MoveSet.RemoveKey(proxyKey + 1);

            if (found)
            {
                // Purge from move buffer. Linear search.
                // todo if I can iterate the move set then I don't need the moveArray
                var count = MoveArray.Count;
                for (var i = 0; i < count; ++i)
                {
                    if (MoveArray[i] == proxyKey)
                    {
                        MoveArray.RemoveSwap(i);
                        break;
                    }
                }
            }
        }

        // This is what triggers new contact pairs to be created
        // Warning: this must be called in deterministic order
        public void BufferMove(int queryProxy)
        {
            // Adding 1 because 0 is the sentinel
            var alreadyAdded = MoveSet.AddKey(queryProxy + 1);
            if (alreadyAdded == false) // Opposite of b2AddKey
            {
                MoveArray.Push(queryProxy);
            }
        }

        public int CreateProxy(
            BodyType proxyType,
            AABB aabb,
            ulong categoryBits,
            int shapeIndex,
            bool forcePairCreation)
        {
            Debug.Assert(0 <= proxyType && proxyType < BodyType.BodyTypeCount);
            int proxyId = Trees[(int)proxyType].CreateProxy(aabb, categoryBits, shapeIndex);
            int proxyKey = ProxyKey(proxyId, proxyType);
            if (proxyType != BodyType.StaticBody || forcePairCreation)
            {
                BufferMove(proxyKey);
            }

            return proxyKey;
        }

        public void DestroyProxy(int proxyKey)
        {
            Debug.Assert(MoveArray.Count == MoveSet.Count);
            UnBufferMove(proxyKey);

            --ProxyCount;

            var proxyType = ProxyType(proxyKey);
            int proxyId = ProxyId(proxyKey);

            Debug.Assert(0 <= proxyType && proxyType <= Core.BodyTypeCount);
            Trees[proxyType].DestroyProxy(proxyId);
        }

        public void MoveProxy(int proxyKey, AABB aabb)
        {
            var proxyType = ProxyType(proxyKey);
            var proxyId = ProxyId(proxyKey);

            Trees[proxyType].MoveProxy(proxyId, aabb);
            BufferMove(proxyKey);
        }

        public void EnlargeProxy(int proxyKey, AABB aabb)
        {
            Debug.Assert(proxyKey != Core.NullIndex);
            var typeIndex = ProxyType(proxyKey);
            var proxyId = ProxyId(proxyKey);

            Debug.Assert(typeIndex != (int)BodyType.StaticBody);

            Trees[typeIndex].EnlargeProxy(proxyId, aabb);
            BufferMove(proxyKey);
        }

        /// <summary>
        /// 移动配对
        /// </summary>
        public class MovePair
        {
            public int ShapeIndexA;

            public int ShapeIndexB;

            /// <summary>
            /// 链表下一个节点
            /// </summary>
            public MovePair? Next;

            public void Reset()
            {
                ShapeIndexA = 0;
                ShapeIndexB = 0;
                Next = null;
            }
        }

        /// <summary>
        /// 移动结果
        /// </summary>
        public struct MoveResult
        {
            public MovePair? PairList;
        }

        public class QueryPairContext
        {
            public World World = null!;

            public MoveResult MoveResult;

            public BodyType QueryTreeType;

            public int QueryProxyKey;

            public int QueryShapeIndex;
        }

        /// <summary>
        /// <summary>
        /// This is called from b2DynamicTree::Query when we are gathering pairs.
        /// 查询动态树，取得重叠配对时触发
        /// </summary>
        /// <returns></returns>
        /// </summary>
        private static readonly TreeQueryCallbackFcn _pairQueryCallback = (proxyId, shapeId, context) =>
        {
            var queryContext = (QueryPairContext)context;
            var bp = queryContext.World.BroadPhase;

            var proxyKey = ProxyKey(proxyId, queryContext.QueryTreeType);

            // A proxy cannot form a pair with itself. 自己与自己不能形成配对
            if (proxyKey == queryContext.QueryProxyKey)
            {
                return true;
            }

            // Is this proxy also moving?
            if (queryContext.QueryTreeType != BodyType.StaticBody)
            {
                var moved = bp.MoveSet.ContainsKey(proxyKey + 1);
                if (moved && proxyKey < queryContext.QueryProxyKey)
                {
                    // Both proxies are moving. Avoid duplicate pairs.
                    return true;
                }
            }

            // 组合被重叠形状Id和当前查询形状Id
            var pairKey = Core.ShapePairKey(shapeId, queryContext.QueryShapeIndex);
            if (bp.PairSet.ContainsKey(pairKey))
            {
                // contact exists
                return true;
            }

            int shapeIdA, shapeIdB;
            if (proxyKey < queryContext.QueryProxyKey)
            {
                shapeIdA = shapeId;
                shapeIdB = queryContext.QueryShapeIndex;
            }
            else
            {
                shapeIdA = queryContext.QueryShapeIndex;
                shapeIdB = shapeId;
            }

            var world = queryContext.World;

            world.ShapeArray.CheckId(shapeIdA);
            world.ShapeArray.CheckId(shapeIdB);

            ref var shapeA = ref world.ShapeArray[shapeIdA];
            ref var shapeB = ref world.ShapeArray[shapeIdB];

            var bodyIdA = shapeA.BodyId;
            var bodyIdB = shapeB.BodyId;

            // Are the shapes on the same body?
            if (bodyIdA == bodyIdB)
            {
                return true;
            }

            if (Contact.ShouldShapesCollide(shapeA.Filter, shapeB.Filter) == false)
            {
                return true;
            }

            // Sensors don't collide with other sensors
            if (shapeA.IsSensor == true && shapeB.IsSensor == true)
            {
                return true;
            }

            // Does a joint override collision?
            var bodyA = Body.GetBody(world, bodyIdA);
            var bodyB = Body.GetBody(world, bodyIdB);
            if (Body.ShouldBodiesCollide(world, bodyA, bodyB) == false)
            {
                return true;
            }

            // Custom user filter
            var customFilterFcn = queryContext.World.CustomFilterFcn;
            if (customFilterFcn != null!)
            {
                ShapeId idA = new(shapeIdA + 1, world.WorldId, shapeA.Revision);
                ShapeId idB = new(shapeIdB + 1, world.WorldId, shapeB.Revision);
                var shouldCollide = customFilterFcn(idA, idB, queryContext.World.CustomFilterContext);
                if (shouldCollide == false)
                {
                    return true;
                }
            }

            // todo per thread to eliminate atomic?
            //var pairIndex = Interlocked.Add(ref bp.MovePairIndex, 1);

            // MovePair pair;
            // if (pairIndex < bp.MovePairCapacity)
            // {
            //     if (bp.MovePairs[pairIndex] == null!)
            //     {
            //         bp.MovePairs[pairIndex] = new();
            //     }
            //
            //     pair = bp.MovePairs[pairIndex];
            // }
            // else
            // {
            //     pair = new();
            // }

            // 执行到这里表示两个形状需要碰撞，创建配对加入链表头，并继续查询重叠
            var pair = new MovePair
            {
                ShapeIndexA = shapeIdA,
                ShapeIndexB = shapeIdB,
                Next = queryContext.MoveResult.PairList
            };
            queryContext.MoveResult.PairList = pair;

            // continue the query
            return true;
        };

        /// <summary>
        /// 查询所有形状AABB重叠，并创建配对对象
        /// </summary>
        /// <param name="startIndex"></param>
        /// <param name="endIndex"></param>
        /// <param name="world"></param>
        public static void FindPairsTask(int startIndex, int endIndex, World world)
        {
            var bp = world.BroadPhase;

            QueryPairContext queryContext = new()
            {
                World = world
            };

            for (var i = startIndex; i < endIndex; ++i)
            {
                // Initialize move result for this moved proxy
                queryContext.MoveResult = bp.MoveResults[i];
                queryContext.MoveResult.PairList = null;

                int proxyKey = bp.MoveArray[i];
                if (proxyKey == Core.NullIndex)
                {
                    // proxy was destroyed after it moved
                    continue;
                }

                var proxyType = ProxyType(proxyKey);
                var proxyId = ProxyId(proxyKey);
                queryContext.QueryProxyKey = proxyKey;

                DynamicTree baseTree = bp.Trees[proxyType];

                // We have to query the tree with the fat AABB so that
                // we don't fail to create a contact that may touch later.
                AABB fatAABB = baseTree.GetAABB(proxyId);
                queryContext.QueryShapeIndex = baseTree.GetUserData(proxyId);

                // Query trees. Only dynamic proxies collide with kinematic and static proxies.
                // Using b2_defaultMaskBits so that b2Filter::groupIndex works.
                // 当前代理是动态刚体时，对运动树和静态树都进行查询，寻找是否存在重叠的AABB
                if ((BodyType)proxyType == BodyType.DynamicBody)
                {
                    queryContext.QueryTreeType = BodyType.KinematicBody;
                    bp.Trees[(int)BodyType.KinematicBody].Query(fatAABB, Core.DefaultMaskBits, _pairQueryCallback, queryContext);
                    bp.MoveResults[i] = queryContext.MoveResult;

                    queryContext.QueryTreeType = BodyType.StaticBody;
                    bp.Trees[(int)BodyType.StaticBody].Query(fatAABB, Core.DefaultMaskBits, _pairQueryCallback, queryContext);
                    bp.MoveResults[i] = queryContext.MoveResult;
                }

                // All proxies collide with dynamic proxies
                // Using b2_defaultMaskBits so that b2Filter::groupIndex works.
                queryContext.QueryTreeType = BodyType.DynamicBody;
                bp.Trees[(int)BodyType.DynamicBody].Query(fatAABB, Core.DefaultMaskBits, _pairQueryCallback, queryContext);
                bp.MoveResults[i] = queryContext.MoveResult;
            }
        }

        public static void UpdateBroadPhasePairs(World world)
        {
            var bp = world.BroadPhase;

            var moveCount = bp.MoveArray.Count;
            Debug.Assert(moveCount == bp.MoveSet.Count);

            if (moveCount == 0)
            {
                return;
            }

            // todo these could be in the step context
            bp.MoveResults = B2ArrayPool<MoveResult>.Shared.Rent(moveCount);
            bp.MoveResultCount = moveCount;

            // bp.MovePairCapacity = 16 * moveCount;
            // bp.MovePairs = B2ArrayPool<MovePair>.Shared.Rent(bp.MovePairCapacity); // b2AllocateStackItem(alloc, bp.movePairCapacity * sizeof(b2MovePair), "move pairs");
            // bp.MovePairIndex = 0;

            // 找到所有形状AABB的重叠配对
            Parallel.ForEach(
                Partitioner.Create(0, moveCount),
                new ParallelOptions() { MaxDegreeOfParallelism = world.WorkerCount },
                range => { FindPairsTask(range.Item1, range.Item2, world); });

            world.TaskCount += 1;

            // Single-threaded work
            // - Clear move flags
            // - Create contacts in deterministic order
            var shapes = world.ShapeArray;

            for (var i = 0; i < moveCount; ++i)
            {
                // 遍历移动结果，对每一个移动结果对应的配对链逐个创建接触点
                ref readonly var result = ref bp.MoveResults[i];
                var pair = result.PairList;
                while (pair != null)
                {
                    // TODO_ERIN Check user filtering.
                    // if (m_contactFilter && m_contactFilter.ShouldCollide(shapeA, shapeB) == false)
                    //{
                    //	return;
                    //}

                    int shapeIdA = pair.ShapeIndexA;
                    int shapeIdB = pair.ShapeIndexB;

                    shapes.CheckId(shapeIdA);
                    shapes.CheckId(shapeIdB);

                    Contact.CreateContact(world, ref shapes[shapeIdA], ref shapes[shapeIdB]);

                    pair = pair.Next;
                }
            }

            // Reset move buffer
            bp.MoveArray.Clear();
            bp.MoveSet.ClearSet();

            // var pairs = bp.MovePairs;
            // bp.MovePairs = null!;
            // B2ArrayPool<MovePair>.Shared.Return(pairs, true);
            var results = bp.MoveResults;
            bp.MoveResults = null!;

            //ArrayHelper.FillFromPool(results, 0, moveCount);

            B2ArrayPool<MoveResult>.Shared.Return(results, true);

            SolverSet.ValidateSolverSets(world);
        }

        public bool TestOverlap(int proxyKeyA, int proxyKeyB)
        {
            int typeIndexA = ProxyType(proxyKeyA);
            int proxyIdA = ProxyId(proxyKeyA);
            int typeIndexB = ProxyType(proxyKeyB);
            int proxyIdB = ProxyId(proxyKeyB);

            AABB aabbA = Trees[typeIndexA].GetAABB(proxyIdA);
            AABB aabbB = Trees[typeIndexB].GetAABB(proxyIdB);
            return AABB.Overlaps(aabbA, aabbB);
        }

        public void RebuildTrees()
        {
            Trees[(int)BodyType.DynamicBody].Rebuild(false);
            Trees[(int)BodyType.KinematicBody].Rebuild(false);
        }

        public int GetShapeIndex(int proxyKey)
        {
            int typeIndex = ProxyType(proxyKey);
            int proxyId = ProxyId(proxyKey);

            return Trees[typeIndex].GetUserData(proxyId);
        }

        public void ValidateBroadphase()
        {
            Trees[(int)BodyType.DynamicBody].Validate();
            Trees[(int)BodyType.KinematicBody].Validate();

            // TODO_ERIN validate every shape AABB is contained in tree AABB
        }

        public void ValidateNoEnlarged()
        {
            if (!Core.B2Validate)
            {
                return;
            }

            for (int j = 0; j < Core.BodyTypeCount; ++j)
            {
                DynamicTree tree = Trees[j];
                int capacity = tree.NodeCapacity;
                var nodes = tree.Nodes.AsSpan();
                for (int i = 0; i < capacity; ++i)
                {
                    ref readonly var node = ref nodes[i];
                    if (node.Height < 0)
                    {
                        continue;
                    }

                    if (node.Enlarged == true)
                    {
                        capacity += 0;
                    }

                    Debug.Assert(node.Enlarged == false);
                }
            }
        }
    }
}