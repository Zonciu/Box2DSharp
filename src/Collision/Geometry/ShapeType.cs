namespace Box2DSharp
{
    /// Shape type
    /// 形状类型
    /// @ingroup shape
    public enum ShapeType
    {
        /// A circle with an offset
        CircleShape,

        /// A capsule is an extruded circle
        CapsuleShape,

        /// A line segment
        SegmentShape,

        /// A convex polygon
        PolygonShape,

        /// A line segment owned by a chain shape
        ChainSegmentShape,

        /// The number of shape types
        ShapeTypeCount
    }
}