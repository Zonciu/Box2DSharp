using System;

namespace Box2DSharp
{
    /// Body events are buffered in the Box2D world and are available
    ///	as event arrays after the time step is complete.
    ///	Note: this data becomes invalid if bodies are destroyed
    public struct BodyEvents
    {
        /// Array of move events
        public Memory<BodyMoveEvent> MoveEvents;

        /// Number of move events
        public int MoveCount;

        public BodyEvents(Memory<BodyMoveEvent> moveEvents, int moveCount)
        {
            MoveEvents = moveEvents;
            MoveCount = moveCount;
        }
    }
}