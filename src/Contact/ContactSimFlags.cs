using System;

namespace Box2DSharp
{
    /// <summary>
    /// Shifted to be distinct from b2ContactFlags
    /// </summary>
    [Flags]
    public enum ContactSimFlags
    {
        /// <summary>
        /// Set when the shapes are touching, including sensors
        /// </summary>
        TouchingFlag = 0x00010000,

        /// <summary>
        /// This contact no longer has overlapping AABBs
        /// </summary>
        Disjoint = 0x00020000,

        /// <summary>
        /// This contact started touching
        /// </summary>
        StartedTouching = 0x00040000,

        /// <summary>
        /// This contact stopped touching
        /// </summary>
        StoppedTouching = 0x00080000,

        /// <summary>
        /// This contact has a hit event
        /// </summary>
        EnableHitEvent = 0x00100000,

        /// <summary>
        /// This contact wants pre-solve events
        /// </summary>
        EnablePreSolveEvents = 0x00200000,
    };
}