namespace Box2DSharp
{
    /**
     * @defgroup events Events
     * World event types.
     *
     * Events are used to collect events that occur during the world time step. These events
     * are then available to query after the time step is complete. This is preferable to callbacks
     * because Box2D uses multithreaded simulation.
     *
     * Also when events occur in the simulation step it may be problematic to modify the world, which is
     * often what applications want to do when events occur.
     *
     * With event arrays, you can scan the events in a loop and modify the world. However, you need to be careful
     * that some event data may become invalid. There are several samples that show how to do this safely.
     *
     * @{
     */
    /// A begin touch event is generated when a shape starts to overlap a sensor shape.
    public struct SensorBeginTouchEvent
    {
        /// The id of the sensor shape
        public ShapeId SensorShapeId;

        /// The id of the dynamic shape that began touching the sensor shape
        public ShapeId VisitorShapeId;

        public SensorBeginTouchEvent(ShapeId sensorShapeId, ShapeId visitorShapeId)
        {
            SensorShapeId = sensorShapeId;
            VisitorShapeId = visitorShapeId;
        }
    }
}