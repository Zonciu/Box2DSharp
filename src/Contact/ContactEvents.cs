using System;

namespace Box2DSharp
{
    /// Contact events are buffered in the Box2D world and are available
    ///	as event arrays after the time step is complete.
    ///	Note: these may become invalid if bodies and/or shapes are destroyed
    public struct ContactEvents
    {
        /// Array of begin touch events
        public Memory<ContactBeginTouchEvent> BeginEvents;

        /// Array of end touch events
        public Memory<ContactEndTouchEvent> EndEvents;

        /// Array of hit events
        public Memory<ContactHitEvent> HitEvents;

        /// Number of begin touch events
        public int BeginCount;

        /// Number of end touch events
        public int EndCount;

        /// Number of hit events
        public int HitCount;

        public ContactEvents(Memory<ContactBeginTouchEvent> beginEvents, Memory<ContactEndTouchEvent> endEvents, Memory<ContactHitEvent> hitEvents, int beginCount, int endCount, int hitCount)
        {
            BeginEvents = beginEvents;
            EndEvents = endEvents;
            HitEvents = hitEvents;
            BeginCount = beginCount;
            EndCount = endCount;
            HitCount = hitCount;
        }
    }
}