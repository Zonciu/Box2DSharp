using System.Diagnostics;

namespace Box2DSharp.Testbed.Unity
{
    public class FpsCounter
    {
        public float Ms = 0;

        public float Fps = 0;

        public int FrameCount;

        private long _lastTimeDiff = 0;

        private long _lastTime = 0;

        private readonly Stopwatch _fpsTimer = Stopwatch.StartNew();

        private float _msSum = 0;

        private int _fpsCount = 0;

        private const int FpsSumMax = 10;

        public void SetFps()
        {
            FrameCount++;
            var now = _fpsTimer.ElapsedMilliseconds;
            _lastTimeDiff = now - _lastTime;
            _lastTime = now;

            if (_fpsCount != FpsSumMax)
            {
                _msSum += _lastTimeDiff;
                _fpsCount++;
            }

            if (_fpsCount == FpsSumMax)
            {
                Ms = _msSum / FpsSumMax;
                Fps = FpsSumMax / (_msSum / 1000f);
                _msSum = 0;
                _fpsCount = 0;
            }
        }
    }
}