using System;
using System.Diagnostics;
using System.Text;
using System.Threading;
using Box2DSharp.Dynamics;
using NETCoreTest.Framework;

namespace NETCoreTest
{
    public class NormalTest
    {
        public FixedUpdate FixedUpdate;

        private Tumbler _tumbler;

        private readonly CancellationTokenSource _stopToken = new CancellationTokenSource();

        public Profile MaxProfile;

        public Profile TotalProfile;

        public bool Pause;

        public bool SingleStep;

        public void Run()
        {
            Console.Clear();
            _tumbler = new Tumbler();
            FixedUpdate = new FixedUpdate {UpdateCallback = Step};
            while (!_stopToken.IsCancellationRequested)
            {
                FixedUpdate.Tick();
            }
        }

        private readonly StringBuilder _sb = new StringBuilder();

        public void Stop()
        {
            _stopToken.Cancel();
        }

        private long _lastOutput;

        private void Step()
        {
            if (Console.KeyAvailable)
            {
                var key = Console.ReadKey(true);
                switch (key.Key)
                {
                case ConsoleKey.P:
                    Pause = !Pause;
                    break;

                case ConsoleKey.O:
                    SingleStep = true;
                    break;
                }
            }

            if (!Pause)
            {
                _tumbler.Step();
            }
            else
            {
                if (SingleStep)
                {
                    SingleStep = false;
                    _tumbler.Step();
                }
            }

            var now = DateTimeOffset.Now.ToUnixTimeMilliseconds();

            var p = _tumbler.World.Profile;

            // Track maximum profile times
            MaxProfile.Step = Math.Max(MaxProfile.Step, p.Step);
            MaxProfile.Collide = Math.Max(MaxProfile.Collide, p.Collide);
            MaxProfile.Solve = Math.Max(MaxProfile.Solve, p.Solve);
            MaxProfile.SolveInit = Math.Max(MaxProfile.SolveInit, p.SolveInit);
            MaxProfile.SolveVelocity = Math.Max(MaxProfile.SolveVelocity, p.SolveVelocity);
            MaxProfile.SolvePosition = Math.Max(MaxProfile.SolvePosition, p.SolvePosition);
            MaxProfile.SolveTOI = Math.Max(MaxProfile.SolveTOI, p.SolveTOI);
            MaxProfile.Broadphase = Math.Max(MaxProfile.Broadphase, p.Broadphase);

            TotalProfile.Step += p.Step;
            TotalProfile.Collide += p.Collide;
            TotalProfile.Solve += p.Solve;
            TotalProfile.SolveInit += p.SolveInit;
            TotalProfile.SolveVelocity += p.SolveVelocity;
            TotalProfile.SolvePosition += p.SolvePosition;
            TotalProfile.SolveTOI += p.SolveTOI;
            TotalProfile.Broadphase += p.Broadphase;

            var aveProfile = new Profile();
            if (FixedUpdate.UpdateTime.FrameCount > 0)
            {
                var scale = 1.0f / FixedUpdate.UpdateTime.FrameCount;
                aveProfile.Step = scale * TotalProfile.Step;
                aveProfile.Collide = scale * TotalProfile.Collide;
                aveProfile.Solve = scale * TotalProfile.Solve;
                aveProfile.SolveInit = scale * TotalProfile.SolveInit;
                aveProfile.SolveVelocity = scale * TotalProfile.SolveVelocity;
                aveProfile.SolvePosition = scale * TotalProfile.SolvePosition;
                aveProfile.SolveTOI = scale * TotalProfile.SolveTOI;
                aveProfile.Broadphase = scale * TotalProfile.Broadphase;
            }

            if (now > _lastOutput)
            {
                _lastOutput = now + 500;
                _sb.AppendLine(Pause ? "****PAUSED****".PadRight(120) : "****RUNNING****".PadRight(120));
                _sb.AppendLine($"FPS {FixedUpdate.UpdateTime.FramePerSecond}, ms {FixedUpdate.UpdateTime.Elapsed.TotalMilliseconds}".PadRight(120));
                _sb.AppendLine($"step [ave] (max) = {p.Step} [{aveProfile.Step}] ({MaxProfile.Step})".PadRight(120));
                _sb.AppendLine($"collide [ave] (max) = {p.Collide} [{aveProfile.Collide}] ({MaxProfile.Collide})".PadRight(120));
                _sb.AppendLine($"solve [ave] (max) = {p.Solve} [{aveProfile.Solve}] ({MaxProfile.Solve})".PadRight(120));
                _sb.AppendLine($"solve init [ave] (max) = {p.SolveInit} [{aveProfile.SolveInit}] ({MaxProfile.SolveInit})".PadRight(120));
                _sb.AppendLine($"solve velocity [ave] (max) = {p.SolveVelocity} [{aveProfile.SolveVelocity}] ({MaxProfile.SolveVelocity})".PadRight(120));
                _sb.AppendLine($"solve position [ave] (max) = {p.SolvePosition} [{aveProfile.SolvePosition}] ({MaxProfile.SolvePosition})".PadRight(120));
                _sb.AppendLine($"solveTOI [ave] (max) = {p.SolveTOI} [{aveProfile.SolveTOI}] ({MaxProfile.SolveTOI})".PadRight(120));
                _sb.AppendLine($"broad-phase [ave] (max) = {p.Broadphase} [{aveProfile.Broadphase}] ({MaxProfile.Broadphase})".PadRight(120));

                Console.SetCursorPosition(0, 0);
                Console.Write(_sb.ToString());
                _sb.Clear();
            }
        }
    }
}