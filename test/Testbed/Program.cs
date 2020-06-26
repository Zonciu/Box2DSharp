using OpenToolkit.Windowing.Desktop;
using OpenToolkit.Mathematics;
using Testbed.Basics;

namespace Testbed
{
    class Program
    {
        static void Main(string[] args)
        {
            Global.Settings.Load();
            Global.Camera.Width = Global.Settings.WindowWidth;
            Global.Camera.Height = Global.Settings.WindowHeight;
            using (var game = new Game(new GameWindowSettings(), new NativeWindowSettings {Size = new Vector2i(Global.Settings.WindowWidth, Global.Settings.WindowHeight)}))
            {
                game.Run();
            }
        }
    }
}