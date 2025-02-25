namespace Box2DSharp
{
    /// <summary>
    /// A hit touch event is generated when two shapes collide with a speed faster than the hit speed threshold.
    /// 当两个形状碰撞的速度快于碰撞速度阈值时，就会产生碰撞触碰事件
    /// </summary>
    public struct ContactHitEvent
    {
        /// Id of the first shape
        public ShapeId ShapeIdA;

        /// Id of the second shape
        public ShapeId ShapeIdB;

        /// Point where the shapes hit
        public Vec2 Point;

        /// Normal vector pointing from shape A to shape B
        public Vec2 Normal;

        /// The speed the shapes are approaching. Always positive. Typically in meters per second.
        public float ApproachSpeed;

        public ContactHitEvent(ShapeId shapeIdA, ShapeId shapeIdB, Vec2 point, Vec2 normal, float approachSpeed)
        {
            ShapeIdA = shapeIdA;
            ShapeIdB = shapeIdB;
            Point = point;
            Normal = normal;
            ApproachSpeed = approachSpeed;
        }
    }
}