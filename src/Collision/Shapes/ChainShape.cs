using System;
using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Common;

namespace Box2DSharp.Collision.Shapes
{
    public class ChainShape : Shape
    {
        /// The vertex count.
        public int Count;

        public bool m_hasPrevVertex, m_hasNextVertex;

        public Vector2 m_prevVertex, m_nextVertex;

        /// The vertices. Owned by this class.
        public Vector2[] Vertices;

        public ChainShape()
        {
            ShapeType       = ShapeType.Chain;
            Radius          = Settings.PolygonRadius;
            Vertices        = null;
            Count           = 0;
            m_hasPrevVertex = false;
            m_hasNextVertex = false;
        }

        /// <summary>
        /// Clear all data.
        /// </summary>
        public void Clear()
        {
            Vertices = null;
            Count    = 0;
        }

        /// Create a loop. This automatically adjusts connectivity.
        /// @param vertices an array of vertices, these are copied
        /// @param count the vertex count
        public void CreateLoop(Vector2[] vertices)
        {
            var count = vertices.Length;
            Debug.Assert(Vertices == null && Count == 0);
            Debug.Assert(count >= 3);
            if (count < 3)
            {
                return;
            }

            for (var i = 1; i < count; ++i)
            {
                var v1 = vertices[i - 1];
                var v2 = vertices[i];

                // If the code crashes here, it means your vertices are too close together.
                Debug.Assert(MathUtils.DistanceSquared(v1, v2) > Settings.LinearSlop * Settings.LinearSlop);
            }

            Count    = count + 1;
            Vertices = new Vector2[Count];
            Array.Copy(vertices, Vertices, count);
            Vertices[count] = Vertices[0];
            m_prevVertex    = Vertices[Count - 2];
            m_nextVertex    = Vertices[1];
            m_hasPrevVertex = true;
            m_hasNextVertex = true;
        }

        /// Create a chain with isolated end vertices.
        /// @param vertices an array of vertices, these are copied
        /// @param count the vertex count
        public void CreateChain(Vector2[] vertices)
        {
            var count = vertices.Length;
            Debug.Assert(Vertices == null && Count == 0);
            Debug.Assert(count >= 2);
            for (var i = 1; i < count; ++i)
            {
                // If the code crashes here, it means your vertices are too close together.
                Debug.Assert(
                    MathUtils.DistanceSquared(vertices[i - 1], vertices[i])
                  > Settings.LinearSlop * Settings.LinearSlop);
            }

            Count    = count;
            Vertices = new Vector2[count];
            Array.Copy(vertices, Vertices, count);

            m_hasPrevVertex = false;
            m_hasNextVertex = false;

            m_prevVertex.SetZero();
            m_nextVertex.SetZero();
        }

        /// Establish connectivity to a vertex that precedes the first vertex.
        /// Don't call this for loops.
        public void SetPrevVertex(in Vector2 prevVertex)
        {
            m_prevVertex    = prevVertex;
            m_hasPrevVertex = true;
        }

        /// Establish connectivity to a vertex that follows the last vertex.
        /// Don't call this for loops.
        public void SetNextVertex(in Vector2 nextVertex)
        {
            m_nextVertex    = nextVertex;
            m_hasNextVertex = true;
        }

        /// Implement b2Shape. Vertices are cloned using b2Alloc.
        public override Shape Clone()
        {
            var clone = new ChainShape();
            clone.CreateChain(Vertices);
            clone.m_prevVertex    = m_prevVertex;
            clone.m_nextVertex    = m_nextVertex;
            clone.m_hasPrevVertex = m_hasPrevVertex;
            clone.m_hasNextVertex = m_hasNextVertex;
            return clone;
        }

        /// @see b2Shape::GetChildCount
        public override int GetChildCount()
        {
            return Count - 1;
        }

        /// Get a child edge.
        public void GetChildEdge(out EdgeShape edge, int index)
        {
            Debug.Assert(0 <= index && index < Count - 1);
            edge = new EdgeShape
            {
                ShapeType = ShapeType.Edge,
                Radius    = Radius,
                Vertex1   = Vertices[index + 0],
                Vertex2   = Vertices[index + 1]
            };

            if (index > 0)
            {
                edge.Vertex0    = Vertices[index - 1];
                edge.HasVertex0 = true;
            }
            else
            {
                edge.Vertex0    = m_prevVertex;
                edge.HasVertex0 = m_hasPrevVertex;
            }

            if (index < Count - 2)
            {
                edge.Vertex3    = Vertices[index + 2];
                edge.HasVertex3 = true;
            }
            else
            {
                edge.Vertex3    = m_nextVertex;
                edge.HasVertex3 = m_hasNextVertex;
            }
        }

        /// This always return false.
        /// @see b2Shape::TestPoint
        public override bool TestPoint(in Transform transform, in Vector2 p)
        {
            return false;
        }

        /// Implement b2Shape.
        public override bool RayCast(
            out RayCastOutput output,
            in  RayCastInput  input,
            in  Transform     transform,
            int               childIndex)
        {
            Debug.Assert(childIndex < Count);

            var edgeShape = new EdgeShape();

            var i1 = childIndex;
            var i2 = childIndex + 1;
            if (i2 == Count)
            {
                i2 = 0;
            }

            edgeShape.Vertex1 = Vertices[i1];
            edgeShape.Vertex2 = Vertices[i2];

            return edgeShape.RayCast(out output, input, transform, 0);
        }

        /// @see b2Shape::ComputeAABB
        public override void ComputeAABB(out AABB aabb, in Transform transform, int childIndex)
        {
            Debug.Assert(childIndex < Count);

            var i1 = childIndex;
            var i2 = childIndex + 1;
            if (i2 == Count)
            {
                i2 = 0;
            }

            var v1 = MathUtils.Mul(transform, Vertices[i1]);
            var v2 = MathUtils.Mul(transform, Vertices[i2]);
            aabb = new AABB(Vector2.Min(v1, v2), Vector2.Max(v1, v2));
        }

        /// Chains have zero mass.
        /// @see b2Shape::ComputeMass
        public override void ComputeMass(out MassData massData, float density)
        {
            massData      = new MassData();
            massData.Mass = 0.0f;
            massData.Center.SetZero();
            massData.RotationInertia = 0.0f;
        }
    }
}