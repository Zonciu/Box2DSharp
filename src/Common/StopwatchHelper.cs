using System;
using System.Diagnostics;

namespace Box2DSharp
{
    public static class StopwatchHelper
    {
        private const long TicksPerMillisecond = 10000;

        private const long TicksPerSecond = TicksPerMillisecond * 1000;

        // performance-counter frequency, in counts per ticks.
        // This can speed up conversion from high frequency performance-counter
        // to ticks.
        private static readonly double _tickFrequency = (double)TicksPerSecond / Stopwatch.Frequency;

        public static TimeSpan GetElapsedTime(long startingTimestamp) =>
            GetElapsedTime(startingTimestamp, Stopwatch.GetTimestamp());

        public static TimeSpan GetElapsedTime(long startingTimestamp, long endingTimestamp) =>
            new TimeSpan((long)((endingTimestamp - startingTimestamp) * _tickFrequency));
    }
}