using System;
using Box2DSharp;
using OpenTK.Mathematics;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.GraphicsLibraryFramework;
using Testbed.Abstractions;

namespace Testbed
{
    class Program
    {
        static void Main(string[] args)
        {
            var version = Core.GetVersion();
            var title = $"Box2D Version {version.Major}.{version.Minor}.{version.Revision} | Runtime Version {Environment.Version}";
            Console.WriteLine(title);

            Global.Settings = TestSettingHelper.Load();
            Global.Camera.Width = Global.Settings.WindowWidth;
            Global.Camera.Height = Global.Settings.WindowHeight;
            Global.Settings.WorkerCount = Math.Min(8, Environment.ProcessorCount);
            using var game = new Game(new GameWindowSettings { UpdateFrequency = 60.0 }, new NativeWindowSettings { ClientSize = new Vector2i(1920, 1080) });
            game.Title = title;
            GLFW.SetErrorCallback(ErrorCallback);

            GLFW.WindowHint(WindowHintInt.ContextVersionMajor, 3);
            GLFW.WindowHint(WindowHintInt.ContextVersionMinor, 3);
            GLFW.WindowHint(WindowHintBool.OpenGLForwardCompat, true);
            GLFW.WindowHint(WindowHintOpenGlProfile.OpenGlProfile, OpenGlProfile.Core);

            // MSAA
            GLFW.WindowHint(WindowHintInt.Samples, 4);

            game.Run();
        }

        private static void ErrorCallback(ErrorCode error, string description)
        {
            Console.WriteLine($"GLFW error: {error}, {description}");
        }
    }
}