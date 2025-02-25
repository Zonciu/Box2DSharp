using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DSharp
{
    /// <summary>
    /// A contact manifold describes the contact points between colliding shapes
    /// 接触流形，描述相碰形状之间的接触点
    /// </summary>
    public struct Manifold
    {
        /// <summary>
        /// The manifold points, up to two are possible in 2D
        /// 流形点，2D中最多2个
        /// </summary>
        public ManifoldPoint Point1;

        /// <summary>
        /// The manifold points, up to two are possible in 2D
        /// 流形点，2D中最多2个
        /// </summary>
        public ManifoldPoint Point2;

        /// <summary>
        /// The unit normal vector in world space, points from shape A to bodyB
        /// 世界空间中的单位法向量，从形状A指向刚体B
        /// </summary>
        public Vec2 Normal;

        /// <summary>
        /// The number of contacts points, will be 0, 1, or 2
        /// 接触点数量，有效值0、1、2
        /// </summary>
        public int PointCount;

        public Span<ManifoldPoint> Points
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => MemoryMarshal.CreateSpan(ref Point1, 2);
        }
    }
}