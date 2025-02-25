namespace Box2DSharp
{
    /// <summary>
    /// A begin touch event is generated when two shapes begin touching.
    /// </summary>
    public struct ContactBeginTouchEvent
    {
        /// Id of the first shape
        public ShapeId ShapeIdA;

        /// Id of the second shape
        public ShapeId ShapeIdB;

        public ContactBeginTouchEvent(ShapeId shapeIdA, ShapeId shapeIdB)
        {
            ShapeIdA = shapeIdA;
            ShapeIdB = shapeIdB;
        }
    }
}