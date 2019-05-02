using System.Diagnostics;

namespace NETCoreTest.Framework
{
    public class FpsCounter
    {
        public float Ms = 0;

        public float Fps = 0;

        public int FrameCount;

        private long _lastTimeDiff = 0;

        private int _lastFrame = 0;

        private long _lastTime = 0;

        private readonly Stopwatch _fpsTimer = Stopwatch.StartNew();

        private float _msSum = 0;

        private float _fpsSum = 0;

        private int _fpsCount = 0;

        private const int FpsSumMax = 10;

        public void CountOne()
        {
            var f = FrameCount;
            FrameCount++;
            var frames = f - _lastFrame;
            _lastFrame = f;
            var now = _fpsTimer.ElapsedMilliseconds;
            _lastTimeDiff = now - _lastTime;
            _lastTime = now;

            float second = _lastTimeDiff / 1000f;
            if (_fpsCount != FpsSumMax)
            {
                _msSum += _lastTimeDiff;
                _fpsSum += frames / second;
                _fpsCount++;
            }

            if (_fpsCount == FpsSumMax)
            {
                Ms = _msSum / FpsSumMax;
                Fps = _fpsSum / FpsSumMax;
                _msSum = 0;
                _fpsSum = 0;
                _fpsCount = 0;
            }
        }
    }
}