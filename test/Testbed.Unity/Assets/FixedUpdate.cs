using System;
using System.Diagnostics;

namespace Box2DSharp.Testbed.Unity
{
    public class FixedUpdate
    {
        private readonly Stopwatch _gameTimer = new Stopwatch();

        public long Dt;

        public long LastTime;

        public long Accumulator;

        public readonly long FixedTime = TimeSpan.FromSeconds(0.25d).Ticks;

        public readonly Action Tick;

        public long TickCount;

        public FixedUpdate(TimeSpan dt, Action action)
        {
            Dt = dt.Ticks;
            Tick = action;
        }

        public void SetDt(TimeSpan dt)
        {
            Dt = dt.Ticks;
        }

        public void Start()
        {
            _gameTimer.Start();
            LastTime = _gameTimer.ElapsedTicks;
        }

        public void Reset()
        {
            LastTime = 0;
            Accumulator = 0;
            TickCount = 0;
            _gameTimer.Reset();
        }

        public void Update()
        {
            var now = _gameTimer.ElapsedTicks;
            var frameTime = now - LastTime;
            if (frameTime > FixedTime)
            {
                frameTime = FixedTime;
            }

            LastTime = now;
            Accumulator += frameTime;

            while (Accumulator >= Dt)
            {
                Tick.Invoke();
                Accumulator -= Dt;
                ++TickCount;
            }
        }
    }
}