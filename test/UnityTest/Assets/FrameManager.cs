using System;
using System.Diagnostics;

namespace Box2DSharp
{
    public class FrameManager : IDisposable
    {
        public FrameManager()
        {
#if UNITY_EDITOR
            UnityEditor.EditorApplication.pauseStateChanged += change =>
            {
                if (change == UnityEditor.PauseState.Paused)
                {
                    this.Pause();
                }
                else
                {
                    this.Resume();
                }
            };
#endif
            _stopwatch.Start();
        }

        /// <summary>
        /// 每帧间隔,单位毫秒
        /// </summary>
        public long Interval
        {
            get => _intervalTick / 10000;
            set => _intervalTick = value * 10000;
        }

        /// <summary>
        /// 每帧间隔tick,单位毫微秒,0.1纳秒,和毫秒相差10000倍
        /// </summary>
        private long _intervalTick;

        /// <summary>
        /// 逻辑帧计数
        /// </summary>
        public int FrameCount { get; private set; }

        /// <summary>
        /// 逻辑帧要触发的任务
        /// </summary>
        public Action Job;

        /// <summary>
        /// 计时器
        /// </summary>
        private Stopwatch _stopwatch = new Stopwatch();

        private long _nextTick = 0L;

        public void Tick()
        {
            if (_stopwatch.IsRunning)
            {
                var now = _stopwatch.ElapsedTicks;
                if (_nextTick != 0)
                {
                    if (now < _nextTick)
                    {
                        return;
                    }

                    Job.Invoke();
                    ++FrameCount;
                    _nextTick += _intervalTick; // 加上间隔时间
                }
                else
                {
                    Job.Invoke();
                    ++FrameCount;
                    _nextTick = now + _intervalTick; // 加上间隔时间,减去执行开销
                }
            }
        }

        /// <summary>
        /// 重置
        /// </summary>
        public void Reset()
        {
            FrameCount = 0;
            _stopwatch.Reset();
        }

        public void Pause()
        {
            _stopwatch.Stop();
        }

        public void Resume()
        {
            _stopwatch.Start();
        }

        public void Dispose()
        {
            _stopwatch?.Stop();
        }
    }
}