using System;
using System.ComponentModel;
using System.Diagnostics;

namespace Box2DSharp
{
    public class Shape
    {
        public int Id;

        public int BodyId;

        public int PrevShapeId;

        public int NextShapeId;

        public ShapeType Type;

        public float Density;

        public float Friction;

        public float Restitution;

        public AABB AABB;

        public AABB FatAABB;

        public Vec2 LocalCentroid;

        public int ProxyKey;

        public Filter Filter;

        public object? UserData;

        public uint CustomColor;

        public UnionShape Union;

        public ushort Revision;

        public bool IsSensor;

        public bool EnableSensorEvents;

        public bool EnableContactEvents;

        public bool EnableHitEvents;

        public bool EnablePreSolveEvents;

        public bool EnlargedAABB;

        public bool IsFast;

        public DistanceProxy MakeShapeDistanceProxy()
        {
            switch (Type)
            {
            case ShapeType.CapsuleShape:
                return DistanceFunc.MakeProxy(Union.Capsule.Points, 2, Union.Capsule.Radius);
            case ShapeType.CircleShape:
                return DistanceFunc.MakeProxy(Union.Circle.Center, Union.Circle.Radius);
            case ShapeType.PolygonShape:
                return DistanceFunc.MakeProxy(Union.Polygon.Vertices, Union.Polygon.Count, Union.Polygon.Radius);
            case ShapeType.SegmentShape:
                return DistanceFunc.MakeProxy(Union.Segment.Points, 2, 0.0f);
            case ShapeType.ChainSegmentShape:
                return DistanceFunc.MakeProxy(Union.ChainSegment.Segment.Points, 2, 0.0f);
            default:
            {
                Debug.Assert(false);
                DistanceProxy empty = new();
                return empty;
            }
            }
        }

        public void UpdateShapeAABBs(Transform transform, BodyType proxyType)
        {
            // Compute a bounding box with a speculative margin
            float speculativeDistance = Core.SpeculativeDistance;
            float aabbMargin = Core.b2_aabbMargin;

            AABB aabb = ComputeShapeAABB(transform);
            aabb.LowerBound.X -= speculativeDistance;
            aabb.LowerBound.Y -= speculativeDistance;
            aabb.UpperBound.X += speculativeDistance;
            aabb.UpperBound.Y += speculativeDistance;
            AABB = aabb;

            // Smaller margin for static bodies. Cannot be zero due to TOI tolerance.
            float margin = proxyType == BodyType.StaticBody ? speculativeDistance : aabbMargin;
            AABB fatAABB;
            fatAABB.LowerBound.X = aabb.LowerBound.X - margin;
            fatAABB.LowerBound.Y = aabb.LowerBound.Y - margin;
            fatAABB.UpperBound.X = aabb.UpperBound.X + margin;
            fatAABB.UpperBound.Y = aabb.UpperBound.Y + margin;
            FatAABB = fatAABB;
        }

        public static Shape CreateShapeInternal<T>(
            World world,
            Body body,
            Transform transform,
            ShapeDef def,
            T geometry,
            ShapeType shapeType)
        {
            Debug.Assert(B2Math.IsValid(def.Density) && def.Density >= 0.0f);
            Debug.Assert(B2Math.IsValid(def.Friction) && def.Friction >= 0.0f);
            Debug.Assert(B2Math.IsValid(def.Restitution) && def.Restitution >= 0.0f);

            var shapeId = world.ShapeIdPool.AllocId();
            if (shapeId == world.ShapeArray.Count)
            {
                world.ShapeArray.Push(new Shape());
            }
            else
            {
                Debug.Assert(world.ShapeArray[shapeId].Id == Core.NullIndex);
            }

            world.ShapeArray.CheckIndex(shapeId);
            Shape shape = world.ShapeArray[shapeId];
            switch (geometry)
            {
            case Capsule capsule:
                shape.Union.Capsule = capsule;
                break;

            case Circle circle:
                shape.Union.Circle = circle;
                break;

            case Polygon polygon:
                shape.Union.Polygon = polygon;
                break;

            case Segment segment:
                shape.Union.Segment = segment;
                break;

            case ChainSegment chainSegment:
                shape.Union.ChainSegment = chainSegment;
                break;

            default:
                Debug.Assert(false);
                break;
            }

            shape.Id = shapeId;
            shape.BodyId = body.Id;
            shape.Type = shapeType;
            shape.Density = def.Density;
            shape.Friction = def.Friction;
            shape.Restitution = def.Restitution;
            shape.Filter = def.Filter;
            shape.UserData = def.UserData;
            shape.CustomColor = def.CustomColor;
            shape.IsSensor = def.IsSensor;
            shape.EnlargedAABB = false;
            shape.EnableSensorEvents = def.EnableSensorEvents;
            shape.EnableContactEvents = def.EnableContactEvents;
            shape.EnableHitEvents = def.EnableHitEvents;
            shape.EnablePreSolveEvents = def.EnablePreSolveEvents;
            shape.IsFast = false;
            shape.ProxyKey = Core.NullIndex;
            shape.LocalCentroid = GetShapeCentroid(shape);
            shape.AABB = new AABB();
            shape.FatAABB = new AABB();
            shape.Revision += 1;

            if (body.SetIndex != SolverSetType.DisabledSet)
            {
                BodyType proxyType = body.Type;
                CreateShapeProxy(shape, world.BroadPhase, proxyType, transform, def.ForceContactCreation || def.IsSensor);
            }

            // Add to shape doubly linked list
            if (body.HeadShapeId != Core.NullIndex)
            {
                world.ShapeArray.CheckId(body.HeadShapeId);
                var headShape = world.ShapeArray[body.HeadShapeId];
                headShape.PrevShapeId = shapeId;
            }

            shape.PrevShapeId = Core.NullIndex;
            shape.NextShapeId = body.HeadShapeId;
            body.HeadShapeId = shapeId;
            body.ShapeCount += 1;

            SolverSet.ValidateSolverSets(world);

            return shape;
        }

        public static ShapeId CreateShape<T>(BodyId bodyId, ShapeDef def, T geometry, ShapeType shapeType)
        {
            def.CheckDef();
            Debug.Assert(B2Math.IsValid(def.Density) && def.Density >= 0.0f);
            Debug.Assert(B2Math.IsValid(def.Friction) && def.Friction >= 0.0f);
            Debug.Assert(B2Math.IsValid(def.Restitution) && def.Restitution >= 0.0f);

            var world = World.GetWorldLocked(bodyId.World0);

            var body = Body.GetBodyFullId(world, bodyId);
            var transform = Body.GetBodyTransformQuick(world, body);

            var shape = CreateShapeInternal(world, body, transform, def, geometry, shapeType);

            if (body.AutomaticMass == true)
            {
                Body.UpdateBodyMassData(world, body);
            }

            SolverSet.ValidateSolverSets(world);

            ShapeId id = new(shape.Id + 1, bodyId.World0, shape.Revision);
            return id;
        }

        public static ShapeId CreateCircleShape(BodyId bodyId, ShapeDef def, Circle circle)

        {
            return CreateShape(bodyId, def, circle, ShapeType.CircleShape);
        }

        public static ShapeId CreateCapsuleShape(BodyId bodyId, ShapeDef def, Capsule capsule)
        {
            var lengthSqr = B2Math.DistanceSquared(capsule.Center1, capsule.Center2);
            if (lengthSqr <= Core.LinearSlop * Core.LinearSlop)
            {
                Circle circle = new(B2Math.Lerp(capsule.Center1, capsule.Center2, 0.5f), (float)capsule.Radius);
                return CreateShape(bodyId, def, circle, ShapeType.CircleShape);
            }

            return CreateShape(bodyId, def, capsule, ShapeType.CapsuleShape);
        }

        public static ShapeId CreatePolygonShape(BodyId bodyId, ShapeDef def, Polygon polygon)

        {
            Debug.Assert(B2Math.IsValid(polygon.Radius) && polygon.Radius >= 0.0f);
            return CreateShape(bodyId, def, polygon, ShapeType.PolygonShape);
        }

        public static ShapeId CreateSegmentShape(BodyId bodyId, ShapeDef def, Segment segment)

        {
            float lengthSqr = B2Math.DistanceSquared(segment.Point1, segment.Point2);
            if (lengthSqr <= Core.LinearSlop * Core.LinearSlop)
            {
                Debug.Assert(false);
                return new ShapeId();
            }

            return CreateShape(bodyId, def, segment, ShapeType.SegmentShape);
        }

        // Destroy a shape on a body. This doesn't need to be called when destroying a body.
        public static void DestroyShapeInternal(World world, Shape shape, Body body, bool wakeBodies)
        {
            int shapeId = shape.Id;

            // Remove the shape from the body's doubly linked list.
            if (shape.PrevShapeId != Core.NullIndex)
            {
                world.ShapeArray.CheckId(shape.PrevShapeId);
                world.ShapeArray[shape.PrevShapeId].NextShapeId = shape.NextShapeId;
            }

            if (shape.NextShapeId != Core.NullIndex)
            {
                world.ShapeArray.CheckId(shape.NextShapeId);
                world.ShapeArray[shape.NextShapeId].PrevShapeId = shape.PrevShapeId;
            }

            if (shapeId == body.HeadShapeId)
            {
                body.HeadShapeId = shape.NextShapeId;
            }

            body.ShapeCount -= 1;

            // Remove from broad-phase.
            DestroyShapeProxy(shape, world.BroadPhase);

            // Destroy any contacts associated with the shape.
            var contactKey = body.HeadContactKey;
            while (contactKey != Core.NullIndex)
            {
                int contactId = contactKey >> 1;
                int edgeIndex = contactKey & 1;

                world.ContactArray.CheckIndex(contactId);
                var contact = world.ContactArray[contactId];
                contactKey = contact.Edges[edgeIndex].NextKey;

                if (contact.ShapeIdA == shapeId || contact.ShapeIdB == shapeId)
                {
                    Contact.DestroyContact(world, contact, wakeBodies);
                }
            }

            // Return shape to free list.
            world.ShapeIdPool.FreeId(shapeId);
            shape.Id = Core.NullIndex;

            SolverSet.ValidateSolverSets(world);
        }

        public static void DestroyShape(ShapeId shapeId)
        {
            var world = World.GetWorldLocked(shapeId.World0);

            int id = shapeId.Index1 - 1;
            world.ShapeArray.CheckIdAndRevision(id, shapeId.Revision);

            Shape shape = world.ShapeArray[id];

            // need to wake bodies because this might be a static body
            Body body = Body.GetBody(world, shape.BodyId);
            DestroyShapeInternal(world, shape, body, true);

            if (body.AutomaticMass == true)
            {
                Body.UpdateBodyMassData(world, body);
            }
        }

        public static ChainId CreateChain(BodyId bodyId, in ChainDef def)
        {
            def.CheckDef();
            Debug.Assert(B2Math.IsValid(def.Friction) && def.Friction >= 0.0f);
            Debug.Assert(B2Math.IsValid(def.Restitution) && def.Restitution >= 0.0f);
            Debug.Assert(def.Count >= 4);

            var world = World.GetWorldLocked(bodyId.World0);

            Body body = Body.GetBodyFullId(world, bodyId);
            Transform transform = Body.GetBodyTransformQuick(world, body);

            int chainId = world.ChainIdPool.AllocId();

            if (chainId == world.ChainArray.Count)
            {
                world.ChainArray.Push(new ChainShape());
            }
            else
            {
                Debug.Assert(world.ChainArray[chainId].Id == Core.NullIndex);
            }

            world.ChainArray.CheckIndex(chainId);
            ref ChainShape chainShape = ref world.ChainArray[chainId];

            chainShape.Id = chainId;
            chainShape.BodyId = body.Id;
            chainShape.NextChainId = body.HeadChainId;
            chainShape.Revision += 1;
            body.HeadChainId = chainId;

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.UserData = def.UserData;
            shapeDef.Restitution = def.Restitution;
            shapeDef.Friction = def.Friction;
            shapeDef.Filter = def.Filter;
            shapeDef.EnableContactEvents = false;
            shapeDef.EnableHitEvents = false;
            shapeDef.EnableSensorEvents = false;

            int n = def.Count;
            var points = def.Points;

            if (def.IsLoop)
            {
                chainShape.Count = n;
                chainShape.ShapeIndices = new int[n];

                ChainSegment chainSegment = new();

                int prevIndex = n - 1;
                for (int i = 0; i < n - 2; ++i)
                {
                    chainSegment.Ghost1 = points[prevIndex];
                    chainSegment.Segment.Point1 = points[i];
                    chainSegment.Segment.Point2 = points[i + 1];
                    chainSegment.Ghost2 = points[i + 2];
                    chainSegment.ChainId = chainId;
                    prevIndex = i;

                    Shape shape = CreateShapeInternal(world, body, transform, shapeDef, chainSegment, ShapeType.ChainSegmentShape);
                    chainShape.ShapeIndices[i] = shape.Id;
                }

                {
                    chainSegment.Ghost1 = points[n - 3];
                    chainSegment.Segment.Point1 = points[n - 2];
                    chainSegment.Segment.Point2 = points[n - 1];
                    chainSegment.Ghost2 = points[0];
                    chainSegment.ChainId = chainId;
                    Shape shape = CreateShapeInternal(world, body, transform, shapeDef, chainSegment, ShapeType.ChainSegmentShape);
                    chainShape.ShapeIndices[n - 2] = shape.Id;
                }

                {
                    chainSegment.Ghost1 = points[n - 2];
                    chainSegment.Segment.Point1 = points[n - 1];
                    chainSegment.Segment.Point2 = points[0];
                    chainSegment.Ghost2 = points[1];
                    chainSegment.ChainId = chainId;
                    Shape shape = CreateShapeInternal(world, body, transform, shapeDef, chainSegment, ShapeType.ChainSegmentShape);
                    chainShape.ShapeIndices[n - 1] = shape.Id;
                }
            }
            else
            {
                chainShape.Count = n - 3;
                chainShape.ShapeIndices = new int[n];

                ChainSegment chainSegment = new();

                for (int i = 0; i < n - 3; ++i)
                {
                    chainSegment.Ghost1 = points[i];
                    chainSegment.Segment.Point1 = points[i + 1];
                    chainSegment.Segment.Point2 = points[i + 2];
                    chainSegment.Ghost2 = points[i + 3];
                    chainSegment.ChainId = chainId;

                    Shape shape = CreateShapeInternal(world, body, transform, shapeDef, chainSegment, ShapeType.ChainSegmentShape);
                    chainShape.ShapeIndices[i] = shape.Id;
                }
            }

            ChainId id = new(chainId + 1, world.WorldId, chainShape.Revision);
            return id;
        }

        public static void DestroyChain(ChainId chainId)
        {
            var world = World.GetWorldLocked(chainId.World0);

            int id = chainId.Index1 - 1;
            world.ChainArray.CheckIdAndRevision(id, chainId.Revision);

            ref ChainShape chain = ref world.ChainArray[id];
            bool wakeBodies = true;

            Body body = Body.GetBody(world, chain.BodyId);

            // Remove the chain from the body's singly linked list.
            var chainIdPtr = body.HeadChainId;
            bool found = false;
            while (chainIdPtr != Core.NullIndex)
            {
                if (chainIdPtr == chain.Id)
                {
                    chainIdPtr = chain.NextChainId;
                    found = true;
                    break;
                }

                chainIdPtr = (world.ChainArray[chainIdPtr].NextChainId);
            }

            Debug.Assert(found == true);
            if (found == false)
            {
                return;
            }

            int count = chain.Count;
            for (int i = 0; i < count; ++i)
            {
                int shapeId = chain.ShapeIndices[i];
                world.ShapeArray.CheckId(shapeId);
                Shape shape = world.ShapeArray[shapeId];
                DestroyShapeInternal(world, shape, body, wakeBodies);
            }

            chain.ShapeIndices = null!;

            // Return chain to free list.
            world.ChainIdPool.FreeId(id);
            chain.Id = Core.NullIndex;

            SolverSet.ValidateSolverSets(world);
        }

        public AABB ComputeShapeAABB(Transform xf)
        {
            switch (Type)
            {
            case ShapeType.CapsuleShape:
                return Geometry.ComputeCapsuleAABB(Union.Capsule, xf);
            case ShapeType.CircleShape:
                return Geometry.ComputeCircleAABB(Union.Circle, xf);
            case ShapeType.PolygonShape:
                return Geometry.ComputePolygonAABB(Union.Polygon, xf);
            case ShapeType.SegmentShape:
                return Geometry.ComputeSegmentAABB(Union.Segment, xf);
            case ShapeType.ChainSegmentShape:
                return Geometry.ComputeSegmentAABB(Union.ChainSegment.Segment, xf);
            default:
            {
                Debug.Assert(false);
                AABB empty = new(xf.P, xf.P);
                return empty;
            }
            }
        }

        public static Vec2 GetShapeCentroid(Shape shape)

        {
            switch (shape.Type)
            {
            case ShapeType.CapsuleShape:
                return B2Math.Lerp(shape.Union.Capsule.Center1, shape.Union.Capsule.Center2, 0.5f);
            case ShapeType.CircleShape:
                return shape.Union.Circle.Center;
            case ShapeType.PolygonShape:
                return shape.Union.Polygon.Centroid;
            case ShapeType.SegmentShape:
                return B2Math.Lerp(shape.Union.Segment.Point1, shape.Union.Segment.Point2, 0.5f);
            case ShapeType.ChainSegmentShape:
                return B2Math.Lerp(shape.Union.ChainSegment.Segment.Point1, shape.Union.ChainSegment.Segment.Point2, 0.5f);
            default:
                return Vec2.Zero;
            }
        }

        /// <summary>
        /// 获取形状周长
        /// todo maybe compute this on shape creation
        /// </summary>
        /// <returns></returns>
        public float GetShapePerimeter()
        {
            switch (Type)
            {
            case ShapeType.CapsuleShape:
                return 2.0f * B2Math.Sub(Union.Capsule.Center1, Union.Capsule.Center2).Length + 2.0f * B2Math.Pi * Union.Capsule.Radius;
            case ShapeType.CircleShape:
                return 2.0f * B2Math.Pi * Union.Circle.Radius;
            case ShapeType.PolygonShape:
            {
                var points = Union.Polygon.Vertices;
                int count = Union.Polygon.Count;
                float perimeter = 2.0f * B2Math.Pi * Union.Polygon.Radius;
                Debug.Assert(count > 0);
                Vec2 prev = points[count - 1];
                for (int i = 0; i < count; ++i)
                {
                    Vec2 next = points[i];
                    perimeter += B2Math.Sub(next, prev).Length;
                    prev = next;
                }

                return perimeter;
            }
            case ShapeType.SegmentShape:
                return 2.0f * B2Math.Sub(Union.Segment.Point1, Union.Segment.Point2).Length;
            case ShapeType.ChainSegmentShape:
                return 2.0f * B2Math.Sub(Union.ChainSegment.Segment.Point1, Union.ChainSegment.Segment.Point2).Length;
            default:
                return 0.0f;
            }
        }

        public MassData ComputeShapeMass()
        {
            switch (Type)
            {
            case ShapeType.CapsuleShape:
                return Geometry.ComputeCapsuleMass(Union.Capsule, Density);
            case ShapeType.CircleShape:
                return Geometry.ComputeCircleMass(Union.Circle, Density);
            case ShapeType.PolygonShape:
                return Geometry.ComputePolygonMass(Union.Polygon, Density);
            default:
            {
                return new MassData();
            }
            }
        }

        public ShapeExtent ComputeShapeExtent(Vec2 localCenter)
        {
            ShapeExtent extent = new();

            switch (Type)
            {
            case ShapeType.CapsuleShape:
            {
                float radius = Union.Capsule.Radius;
                extent.MinExtent = radius;
                Vec2 c1 = B2Math.Sub(Union.Capsule.Center1, localCenter);
                Vec2 c2 = B2Math.Sub(Union.Capsule.Center2, localCenter);
                extent.MaxExtent = MathF.Sqrt(Math.Max(B2Math.LengthSquared(c1), B2Math.LengthSquared(c2))) + radius;
            }
                break;

            case ShapeType.CircleShape:
            {
                float radius = Union.Circle.Radius;
                extent.MinExtent = radius;
                extent.MaxExtent = B2Math.Sub(Union.Circle.Center, localCenter).Length + radius;
            }
                break;

            case ShapeType.PolygonShape:
            {
                var poly = Union.Polygon;
                float minExtent = Core.Huge;
                float maxExtentSqr = 0.0f;
                int count = poly.Count;
                for (int i = 0; i < count; ++i)
                {
                    Vec2 v = poly.Vertices[i];
                    float planeOffset = B2Math.Dot(poly.Normals[i], B2Math.Sub(v, poly.Centroid));
                    minExtent = Math.Min(minExtent, planeOffset);

                    float distanceSqr = B2Math.LengthSquared(B2Math.Sub(v, localCenter));
                    maxExtentSqr = Math.Max(maxExtentSqr, distanceSqr);
                }

                extent.MinExtent = minExtent + poly.Radius;
                extent.MaxExtent = MathF.Sqrt(maxExtentSqr) + poly.Radius;
            }
                break;

            case ShapeType.SegmentShape:
            {
                extent.MinExtent = 0.0f;
                Vec2 c1 = B2Math.Sub(Union.Segment.Point1, localCenter);
                Vec2 c2 = B2Math.Sub(Union.Segment.Point2, localCenter);
                extent.MaxExtent = MathF.Sqrt(Math.Max(B2Math.LengthSquared(c1), B2Math.LengthSquared(c2)));
            }
                break;

            case ShapeType.ChainSegmentShape:
            {
                extent.MinExtent = 0.0f;
                Vec2 c1 = B2Math.Sub(Union.ChainSegment.Segment.Point1, localCenter);
                Vec2 c2 = B2Math.Sub(Union.ChainSegment.Segment.Point2, localCenter);
                extent.MaxExtent = MathF.Sqrt(Math.Max(B2Math.LengthSquared(c1), B2Math.LengthSquared(c2)));
            }
                break;
            default:
                throw new InvalidEnumArgumentException($"Shape type {Type} is invalid");
            }

            return extent;
        }

        public static CastOutput RayCastShape(in RayCastInput input, Shape shape, Transform transform)
        {
            RayCastInput localInput = input;
            localInput.Origin = B2Math.InvTransformPoint(transform, input.Origin);
            localInput.Translation = B2Math.InvRotateVector(transform.Q, input.Translation);

            CastOutput output = new();
            switch (shape.Type)
            {
            case ShapeType.CapsuleShape:
                output = Geometry.RayCastCapsule(localInput, shape.Union.Capsule);
                break;
            case ShapeType.CircleShape:
                output = Geometry.RayCastCircle(localInput, shape.Union.Circle);
                break;
            case ShapeType.PolygonShape:
                output = Geometry.RayCastPolygon(localInput, shape.Union.Polygon);
                break;
            case ShapeType.SegmentShape:
                output = Geometry.RayCastSegment(localInput, shape.Union.Segment, false);
                break;
            case ShapeType.ChainSegmentShape:
                output = Geometry.RayCastSegment(localInput, shape.Union.ChainSegment.Segment, true);
                break;
            default:
                return output;
            }

            output.Point = B2Math.TransformPoint(transform, output.Point);
            output.Normal = B2Math.RotateVector(transform.Q, output.Normal);
            return output;
        }

        public static CastOutput ShapeCastShape(ShapeCastInput input, Shape shape, Transform transform)
        {
            ShapeCastInput localInput = input;

            for (int i = 0; i < localInput.Count; ++i)
            {
                localInput.Points[i] = B2Math.InvTransformPoint(transform, input.Points[i]);
            }

            localInput.Translation = B2Math.InvRotateVector(transform.Q, input.Translation);

            CastOutput output = new();
            switch (shape.Type)
            {
            case ShapeType.CapsuleShape:
                output = Geometry.ShapeCastCapsule(localInput, shape.Union.Capsule);
                break;
            case ShapeType.CircleShape:
                output = Geometry.ShapeCastCircle(localInput, shape.Union.Circle);
                break;
            case ShapeType.PolygonShape:
                output = Geometry.ShapeCastPolygon(localInput, shape.Union.Polygon);
                break;
            case ShapeType.SegmentShape:
                output = Geometry.ShapeCastSegment(localInput, shape.Union.Segment);
                break;
            case ShapeType.ChainSegmentShape:
                output = Geometry.ShapeCastSegment(localInput, shape.Union.ChainSegment.Segment);
                break;
            default:
                return output;
            }

            output.Point = B2Math.TransformPoint(transform, output.Point);
            output.Normal = B2Math.RotateVector(transform.Q, output.Normal);
            return output;
        }

        public static void CreateShapeProxy(Shape shape, BroadPhase bp, BodyType bodyType, Transform transform, bool forcePairCreation)
        {
            Debug.Assert(shape.ProxyKey == Core.NullIndex);

            shape.UpdateShapeAABBs(transform, bodyType);

            // Create proxies in the broad-phase.
            shape.ProxyKey = bp.CreateProxy(bodyType, shape.FatAABB, shape.Filter.CategoryBits, shape.Id, forcePairCreation);
            Debug.Assert(BroadPhase.ProxyType(shape.ProxyKey) < Core.BodyTypeCount);
        }

        public static void DestroyShapeProxy(Shape shape, BroadPhase bp)
        {
            if (shape.ProxyKey != Core.NullIndex)
            {
                bp.DestroyProxy(shape.ProxyKey);
                shape.ProxyKey = Core.NullIndex;
            }
        }

        public static DistanceProxy MakeShapeDistanceProxy(Shape shape)
        {
            switch (shape.Type)
            {
            case ShapeType.CapsuleShape:
                return DistanceFunc.MakeProxy(shape.Union.Capsule.Points, 2, shape.Union.Capsule.Radius);
            case ShapeType.CircleShape:
                return DistanceFunc.MakeProxy(shape.Union.Circle.Center, shape.Union.Circle.Radius);
            case ShapeType.PolygonShape:
                return DistanceFunc.MakeProxy(shape.Union.Polygon.Vertices, shape.Union.Polygon.Count, shape.Union.Polygon.Radius);
            case ShapeType.SegmentShape:
                return DistanceFunc.MakeProxy(shape.Union.Segment.Points, 2, 0.0f);
            case ShapeType.ChainSegmentShape:
                return DistanceFunc.MakeProxy(shape.Union.ChainSegment.Segment.Points, 2, 0.0f);
            default:
            {
                Debug.Assert(false);
                DistanceProxy empty = new();
                return empty;
            }
            }
        }

        public static BodyId GetBody(ShapeId shapeId)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            return Body.MakeBodyId(world, shape.BodyId);
        }

        public static void SetUserData(ShapeId shapeId, object? userData)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            shape.UserData = userData;
        }

        public static object? GetUserData(ShapeId shapeId)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            return shape.UserData;
        }

        public static bool GetIsSensor(ShapeId shapeId)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            return shape.IsSensor;
        }

        public static bool TestPoint(ShapeId shapeId, Vec2 point)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);

            Transform transform = Body.GetBodyTransform(world, shape.BodyId);
            Vec2 localPoint = B2Math.InvTransformPoint(transform, point);

            switch (shape.Type)
            {
            case ShapeType.CapsuleShape:
                return Geometry.PointInCapsule(localPoint, shape.Union.Capsule);

            case ShapeType.CircleShape:
                return Geometry.PointInCircle(localPoint, shape.Union.Circle);

            case ShapeType.PolygonShape:
                return Geometry.PointInPolygon(localPoint, shape.Union.Polygon);

            default:
                return false;
            }
        }

        // todo_erin untested
        public static CastOutput RayCast(ShapeId shapeId, RayCastInput input)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);

            Transform transform = Body.GetBodyTransform(world, shape.BodyId);

            // input in local coordinates
            RayCastInput localInput;
            localInput.Origin = B2Math.InvTransformPoint(transform, input.Origin);
            localInput.Translation = B2Math.InvRotateVector(transform.Q, input.Translation);
            localInput.MaxFraction = input.MaxFraction;

            CastOutput output = new();
            switch (shape.Type)
            {
            case ShapeType.CapsuleShape:
                output = Geometry.RayCastCapsule(localInput, shape.Union.Capsule);
                break;

            case ShapeType.CircleShape:
                output = Geometry.RayCastCircle(localInput, shape.Union.Circle);
                break;

            case ShapeType.SegmentShape:
                output = Geometry.RayCastSegment(localInput, shape.Union.Segment, false);
                break;

            case ShapeType.PolygonShape:
                output = Geometry.RayCastPolygon(localInput, shape.Union.Polygon);
                break;

            case ShapeType.ChainSegmentShape:
                output = Geometry.RayCastSegment(localInput, shape.Union.ChainSegment.Segment, true);
                break;

            default:
                Debug.Assert(false);
                return output;
            }

            if (output.Hit)
            {
                // convert to world coordinates
                output.Normal = B2Math.RotateVector(transform.Q, output.Normal);
                output.Point = B2Math.TransformPoint(transform, output.Point);
            }

            return output;
        }

        public static void SetDensity(ShapeId shapeId, float density)
        {
            Debug.Assert(B2Math.IsValid(density) && density >= 0.0f);

            var world = World.GetWorldLocked(shapeId.World0);

            Shape shape = world.GetShape(shapeId);
            if (density == shape.Density)
            {
                // early return to avoid expensive function
                return;
            }

            shape.Density = density;
        }

        public static float GetDensity(ShapeId shapeId)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            return shape.Density;
        }

        public static void SetFriction(ShapeId shapeId, float friction)
        {
            Debug.Assert(B2Math.IsValid(friction) && friction >= 0.0f);

            World world = World.GetWorld(shapeId.World0);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return;
            }

            Shape shape = world.GetShape(shapeId);
            shape.Friction = friction;
        }

        public static float GetFriction(ShapeId shapeId)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            return shape.Friction;
        }

        public static void SetRestitution(ShapeId shapeId, float restitution)
        {
            Debug.Assert(B2Math.IsValid(restitution) && restitution >= 0.0f);

            World world = World.GetWorld(shapeId.World0);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return;
            }

            Shape shape = world.GetShape(shapeId);
            shape.Restitution = restitution;
        }

        public static float GetRestitution(ShapeId shapeId)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            return shape.Restitution;
        }

        public static Filter GetFilter(ShapeId shapeId)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            return shape.Filter;
        }

        public static void ResetProxy(World world, Shape shape, bool wakeBodies, bool destroyProxy)
        {
            Body body = Body.GetBody(world, shape.BodyId);

            int shapeId = shape.Id;

            // destroy all contacts associated with this shape
            int contactKey = body.HeadContactKey;
            while (contactKey != Core.NullIndex)
            {
                int contactId = contactKey >> 1;
                int edgeIndex = contactKey & 1;

                world.ContactArray.CheckIndex(contactId);
                var contact = world.ContactArray[contactId];
                contactKey = contact.Edges[edgeIndex].NextKey;

                if (contact.ShapeIdA == shapeId || contact.ShapeIdB == shapeId)
                {
                    Contact.DestroyContact(world, contact, wakeBodies);
                }
            }

            Transform transform = Body.GetBodyTransformQuick(world, body);
            if (shape.ProxyKey != Core.NullIndex)
            {
                var proxyType = BroadPhase.ProxyType(shape.ProxyKey);
                shape.UpdateShapeAABBs(transform, (BodyType)proxyType);

                if (destroyProxy)
                {
                    world.BroadPhase.DestroyProxy(shape.ProxyKey);

                    bool forcePairCreation = true;
                    shape.ProxyKey =
                        world.BroadPhase.CreateProxy(
                            (BodyType)proxyType,
                            shape.FatAABB,
                            shape.Filter.CategoryBits,
                            shapeId,
                            forcePairCreation);
                }
                else
                {
                    world.BroadPhase.MoveProxy(shape.ProxyKey, shape.FatAABB);
                }
            }
            else
            {
                BodyType proxyType = body.Type;
                shape.UpdateShapeAABBs(transform, proxyType);
            }

            SolverSet.ValidateSolverSets(world);
        }

        public static void SetFilter(ShapeId shapeId, Filter filter)
        {
            var world = World.GetWorldLocked(shapeId.World0);

            Shape shape = world.GetShape(shapeId);
            if (filter.MaskBits == shape.Filter.MaskBits && filter.CategoryBits == shape.Filter.CategoryBits && filter.GroupIndex == shape.Filter.GroupIndex)
            {
                return;
            }

            // If the category bits change, I need to destroy the proxy because it affects the tree sorting.
            bool destroyProxy = filter.CategoryBits == shape.Filter.CategoryBits;

            shape.Filter = filter;

            // need to wake bodies because a filter change may destroy contacts
            bool wakeBodies = true;
            ResetProxy(world, shape, wakeBodies, destroyProxy);
        }

        public static void SetEnableSensorEvents(ShapeId shapeId, bool flag)
        {
            var world = World.GetWorldLocked(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            shape.EnableSensorEvents = flag;
        }

        public static bool AreSensorEventsEnabled(ShapeId shapeId)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            return shape.EnableSensorEvents;
        }

        public static void SetEnableContactEvents(ShapeId shapeId, bool flag)
        {
            var world = World.GetWorldLocked(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            shape.EnableContactEvents = flag;
        }

        public static bool AreContactEventsEnabled(ShapeId shapeId)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            return shape.EnableContactEvents;
        }

        public static void SetEnablePreSolveEvents(ShapeId shapeId, bool flag)
        {
            var world = World.GetWorldLocked(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            shape.EnablePreSolveEvents = flag;
        }

        public static bool ArePreSolveEventsEnabled(ShapeId shapeId)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            return shape.EnablePreSolveEvents;
        }

        public static void SetEnableHitEvents(ShapeId shapeId, bool flag)
        {
            var world = World.GetWorldLocked(shapeId.World0);

            Shape shape = world.GetShape(shapeId);
            shape.EnableHitEvents = flag;
        }

        public bool AreHitEventsEnabled(ShapeId shapeId)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            return shape.EnableHitEvents;
        }

        public static ShapeType GetType(ShapeId shapeId)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            return shape.Type;
        }

        public static Circle GetCircle(ShapeId shapeId)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            Debug.Assert(shape.Type == ShapeType.CircleShape);
            return shape.Union.Circle;
        }

        public static Segment GetSegment(ShapeId shapeId)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            Debug.Assert(shape.Type == ShapeType.SegmentShape);
            return shape.Union.Segment;
        }

        public static ChainSegment GetChainSegment(ShapeId shapeId)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            Debug.Assert(shape.Type == ShapeType.ChainSegmentShape);
            return shape.Union.ChainSegment;
        }

        public static Capsule GetCapsule(ShapeId shapeId)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            Debug.Assert(shape.Type == ShapeType.CapsuleShape);
            return shape.Union.Capsule;
        }

        public static Polygon GetPolygon(ShapeId shapeId)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            Debug.Assert(shape.Type == ShapeType.PolygonShape);
            return shape.Union.Polygon;
        }

        public static void SetCircle(ShapeId shapeId, Circle circle)
        {
            var world = World.GetWorldLocked(shapeId.World0);

            Shape shape = world.GetShape(shapeId);
            shape.Union.Circle = circle;
            shape.Type = ShapeType.CircleShape;

            // need to wake bodies so they can react to the shape change
            bool wakeBodies = true;
            bool destroyProxy = true;
            ResetProxy(world, shape, wakeBodies, destroyProxy);
        }

        public static void SetCapsule(ShapeId shapeId, Capsule capsule)
        {
            var world = World.GetWorldLocked(shapeId.World0);

            Shape shape = world.GetShape(shapeId);
            shape.Union.Capsule = capsule;
            shape.Type = ShapeType.CapsuleShape;

            // need to wake bodies so they can react to the shape change
            bool wakeBodies = true;
            bool destroyProxy = true;
            ResetProxy(world, shape, wakeBodies, destroyProxy);
        }

        public static void SetSegment(ShapeId shapeId, Segment segment)
        {
            var world = World.GetWorldLocked(shapeId.World0);

            Shape shape = world.GetShape(shapeId);
            shape.Union.Segment = segment;
            shape.Type = ShapeType.SegmentShape;

            // need to wake bodies so they can react to the shape change
            bool wakeBodies = true;
            bool destroyProxy = true;
            ResetProxy(world, shape, wakeBodies, destroyProxy);
        }

        public static void SetPolygon(ShapeId shapeId, Polygon polygon)
        {
            var world = World.GetWorldLocked(shapeId.World0);

            Shape shape = world.GetShape(shapeId);
            shape.Union.Polygon = polygon;
            shape.Type = ShapeType.PolygonShape;

            // need to wake bodies so they can react to the shape change
            bool wakeBodies = true;
            bool destroyProxy = true;
            ResetProxy(world, shape, wakeBodies, destroyProxy);
        }

        public static ChainId GetParentChain(ShapeId shapeId)
        {
            World world = World.GetWorld(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            if (shape.Type == ShapeType.ChainSegmentShape)
            {
                int chainId = shape.Union.ChainSegment.ChainId;
                if (chainId != Core.NullIndex)
                {
                    world.ChainArray.CheckId(chainId);
                    ref ChainShape chain = ref world.ChainArray[chainId];
                    ChainId id = new(chainId + 1, shapeId.World0, chain.Revision);
                    return id;
                }
            }

            return new ChainId();
        }

        public static void SetFriction(ChainId chainId, float friction)
        {
            var world = World.GetWorldLocked(chainId.World0);

            ref ChainShape chainShape = ref world.GetChainShape(chainId);

            int count = chainShape.Count;

            for (int i = 0; i < count; ++i)
            {
                int shapeId = chainShape.ShapeIndices[i];
                world.ShapeArray.CheckId(shapeId);
                Shape shape = world.ShapeArray[shapeId];
                shape.Friction = friction;
            }
        }

        public static void SetRestitution(ChainId chainId, float restitution_)
        {
            var world = World.GetWorldLocked(chainId.World0);
            ref ChainShape chainShape = ref world.GetChainShape(chainId);

            int count = chainShape.Count;

            for (int i = 0; i < count; ++i)
            {
                int shapeId = chainShape.ShapeIndices[i];
                world.ShapeArray.CheckId(shapeId);
                Shape shape = world.ShapeArray[shapeId];
                shape.Restitution = restitution_;
            }
        }

        public static int GetContactCapacity(ShapeId shapeId)
        {
            var world = World.GetWorldLocked(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            if (shape.IsSensor)
            {
                return 0;
            }

            Body body = Body.GetBody(world, shape.BodyId);

            // Conservative and fast
            return body.ContactCount;
        }

        // todo sample needed
        public static int GetContactData(ShapeId shapeId, Span<ContactData> contactData, int capacity)
        {
            var world = World.GetWorldLocked(shapeId.World0);
            Shape shape = world.GetShape(shapeId);
            if (shape.IsSensor)
            {
                return 0;
            }

            Body body = Body.GetBody(world, shape.BodyId);
            int contactKey = body.HeadContactKey;
            int index = 0;
            while (contactKey != Core.NullIndex && index < capacity)
            {
                int contactId = contactKey >> 1;
                int edgeIndex = contactKey & 1;

                world.ContactArray.CheckIndex(contactId);
                Contact contact = world.ContactArray[contactId];

                // Does contact involve this shape and is it touching?
                if ((contact.ShapeIdA == shapeId.Index1 - 1 || contact.ShapeIdB == shapeId.Index1 - 1) && (contact.Flags.IsSet(ContactFlags.ContactTouchingFlag)))
                {
                    Shape shapeA = world.ShapeArray[contact.ShapeIdA];
                    Shape shapeB = world.ShapeArray[contact.ShapeIdB];

                    contactData[index].ShapeIdA = new ShapeId(shapeA.Id + 1, shapeId.World0, shapeA.Revision)
                        ;
                    contactData[index].ShapeIdB = new ShapeId(shapeB.Id + 1, shapeId.World0, shapeB.Revision)
                        ;

                    ContactSim contactSim = Contact.GetContactSim(world, contact);
                    contactData[index].Manifold = contactSim.Manifold;
                    index += 1;
                }

                contactKey = contact.Edges[edgeIndex].NextKey;
            }

            Debug.Assert(index <= capacity);

            return index;
        }

        public static AABB GetAABB(ShapeId shapeId)
        {
            var world = World.GetWorld(shapeId.World0);

            Shape shape = world.GetShape(shapeId);
            return shape.AABB;
        }

        public static Vec2 GetClosestPoint(ShapeId shapeId, Vec2 target)
        {
            World world = World.GetWorld(shapeId.World0);

            Shape shape = world.GetShape(shapeId);
            Body body = Body.GetBody(world, shape.BodyId);
            Transform transform = Body.GetBodyTransformQuick(world, body);

            DistanceInput input;
            input.ProxyA = MakeShapeDistanceProxy(shape);
            input.ProxyB = DistanceFunc.MakeProxy(target, 0.0f);
            input.TransformA = transform;
            input.TransformB = Transform.Identity;
            input.UseRadii = true;

            DistanceCache cache = new();
            DistanceOutput output = DistanceFunc.ShapeDistance(ref cache, input, null, 0);

            return output.PointA;
        }
    }
}