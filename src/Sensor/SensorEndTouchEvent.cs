namespace Box2DSharp
{
    /// An end touch event is generated when a shape stops overlapping a sensor shape.
    public struct SensorEndTouchEvent
    {
        /// The id of the sensor shape
        public ShapeId SensorShapeId;

        /// The id of the dynamic shape that stopped touching the sensor shape
        public ShapeId VisitorShapeId;

        public SensorEndTouchEvent(ShapeId sensorShapeId, ShapeId visitorShapeId)
        {
            SensorShapeId = sensorShapeId;
            VisitorShapeId = visitorShapeId;
        }
    }
}