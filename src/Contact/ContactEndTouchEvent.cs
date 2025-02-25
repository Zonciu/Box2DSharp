namespace Box2DSharp
{
    /// <summary>
    /// An end touch event is generated when two shapes stop touching.
    /// </summary>
    public struct ContactEndTouchEvent
    {
        /// Id of the first shape
        public ShapeId ShapeIdA;

        /// Id of the second shape
        public ShapeId ShapeIdB;

        public ContactEndTouchEvent(ShapeId shapeIdA, ShapeId shapeIdB)
        {
            ShapeIdA = shapeIdA;
            ShapeIdB = shapeIdB;
        }
    }
}