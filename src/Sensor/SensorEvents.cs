using System;

namespace Box2DSharp
{
    /// Sensor events are buffered in the Box2D world and are available
    ///	as begin/end overlap event arrays after the time step is complete.
    ///	Note: these may become invalid if bodies and/or shapes are destroyed
    public struct SensorEvents
    {
        /// Array of sensor begin touch events
        public Memory<SensorBeginTouchEvent> BeginEvents;

        /// Array of sensor end touch events
        public Memory<SensorEndTouchEvent> EndEvents;

        /// The number of begin touch events
        public int BeginCount;

        /// The number of end touch events
        public int EndCount;

        public SensorEvents(Memory<SensorBeginTouchEvent> beginEvents, Memory<SensorEndTouchEvent> endEvents, int beginCount, int endCount)
        {
            BeginEvents = beginEvents;
            EndEvents = endEvents;
            BeginCount = beginCount;
            EndCount = endCount;
        }
    }
}