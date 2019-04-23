using System;
using System.Diagnostics;

namespace Box2DSharp
{
    public class FixedUpdate
    {
        private readonly Stopwatch _gameTimer = new Stopwatch();

        public long Dt;

        public long CurrentTime;

        public long Accumulator;

        public readonly long FixedTime = TimeSpan.FromSeconds(0.25d).Ticks;

        public event Action Tick;

        public long TickCount;

        public FixedUpdate(TimeSpan dt, Action action)
        {
            Dt = dt.Ticks;
            Tick += action;
        }

        public void SetDt(TimeSpan dt)
        {
            Dt = dt.Ticks;
        }

        public void Start()
        {
            _gameTimer.Start();
            CurrentTime = _gameTimer.ElapsedTicks;
        }

        public void Reset()
        {
            CurrentTime = 0;
            Accumulator = 0;
            TickCount = 0;
            _gameTimer.Reset();
        }

        public void Update(TimeSpan? dt = default)
        {
            var newTime = _gameTimer.ElapsedTicks;
            var frameTime = newTime - CurrentTime;
            if (frameTime > FixedTime)
            {
                frameTime = FixedTime;
            }

            CurrentTime = newTime;
            Accumulator += frameTime;
            var runDt = Dt;
            if (dt.HasValue)
            {
                runDt = dt.Value.Ticks;
            }

            while (Accumulator >= runDt)
            {
                Tick?.Invoke();
                Accumulator -= runDt;
                ++TickCount;
            }
        }
    }
}