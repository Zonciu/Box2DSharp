using System;
using System.Diagnostics;
using Box2DSharp.Common;
using ImGuiNET;
using OpenToolkit.Graphics.OpenGL4;
using OpenToolkit.Mathematics;
using OpenToolkit.Windowing.Common;
using OpenToolkit.Windowing.Common.Input;
using OpenToolkit.Windowing.Desktop;
using Testbed.Basics;
using Testbed.Basics.ImGui;

namespace Testbed
{
    public class Game : GameWindow
    {
        private ImGuiController _controller;

        private long _frameTime;

        private long _lastUpdateTime;

        private Stopwatch _stopwatch = Stopwatch.StartNew();

        public Test Test;

        /// <inheritdoc />
        public Game(GameWindowSettings gameWindowSettings, NativeWindowSettings nativeWindowSettings)
            : base(gameWindowSettings, nativeWindowSettings)
        { }

        /// <inheritdoc />
        protected override void OnLoad()
        {
            GL.ClearColor(0.2f, 0.2f, 0.2f, 1.0f);
            _controller = new ImGuiController(Size.X, Size.Y);
            Global.Settings.Load();
            Global.DebugDraw.Create();
            _currentTestIndex = Math.Clamp(_currentTestIndex, 0, Global.Tests.Count - 1);
            _testSelection = _currentTestIndex;
            RestartTest();
            base.OnLoad();
        }

        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            var input = KeyboardState;
            if (input.IsKeyDown(Key.Escape))
            {
                Close();
            }

            CheckTestChange();
            Test.Step();
            var now = _stopwatch.ElapsedTicks;
            _frameTime = now - _lastUpdateTime;
            _lastUpdateTime = now;
            base.OnUpdateFrame(e);
        }

        static float _avgDuration = 0;

        const float Alpha = 1f / 100f; // 采样数设置为100

        static int _frameCount = 0;

        private int GetFps(float deltaTime) // ms
        {
            ++_frameCount;

            if (1 == _frameCount)
            {
                _avgDuration = deltaTime;
            }
            else
            {
                _avgDuration = _avgDuration * (1 - Alpha) + deltaTime * Alpha;
            }

            return (int)(1f / _avgDuration * 1000);
        }

        protected override void OnRenderFrame(FrameEventArgs e)
        {
            _controller.Update(this, (float)e.Time);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
            UpdateText();
            UpdateUI();
            Test.Render();
            if (Global.DebugDraw.ShowUI)
            {
                Global.DebugDraw.DrawString(5, Global.Camera.Height - 60, $"steps: {Test.StepCount}");
                Global.DebugDraw.DrawString(5, Global.Camera.Height - 40, $"{_frameTime / 10000f:.#} ms");
                Global.DebugDraw.DrawString(5, Global.Camera.Height - 20, $"{GetFps(_frameTime / 10000f)} fps");
            }

            Global.DebugDraw.Flush();

            _controller.Render();

            Util.CheckGLError("End of frame");
            SwapBuffers();
            base.OnRenderFrame(e);
        }

        private void UpdateText()
        {
            if (Global.DebugDraw.ShowUI)
            {
                ImGui.SetNextWindowPos(new System.Numerics.Vector2(0.0f, 0.0f));
                ImGui.SetNextWindowSize(new System.Numerics.Vector2(Global.Camera.Width, Global.Camera.Height));
                ImGui.SetNextWindowBgAlpha(0);
                ImGui.Begin("Overlay", ImGuiWindowFlags.NoTitleBar | ImGuiWindowFlags.NoInputs | ImGuiWindowFlags.AlwaysAutoResize | ImGuiWindowFlags.NoScrollbar);
                ImGui.End();
                var (category, name, _) = Global.Tests[_currentTestIndex];
                Test?.DrawTitle($"{category} : {name}");
            }
        }

        public void UpdateUI()
        {
            const int MenuWidth = 180;
            if (Global.DebugDraw.ShowUI)
            {
                ImGui.SetNextWindowPos(new System.Numerics.Vector2((float)Global.Camera.Width - MenuWidth - 10, 10));
                ImGui.SetNextWindowSize(new System.Numerics.Vector2(MenuWidth, (float)Global.Camera.Height - 20));

                ImGui.Begin("Tools", ref Global.DebugDraw.ShowUI, ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoCollapse);

                if (ImGui.BeginTabBar("ControlTabs", ImGuiTabBarFlags.None))
                {
                    if (ImGui.BeginTabItem("Controls"))
                    {
                        ImGui.SliderInt("Vel Iters", ref Global.Settings.VelocityIterations, 0, 50);
                        ImGui.SliderInt("Pos Iters", ref Global.Settings.PositionIterations, 0, 50);
                        ImGui.SliderFloat("Hertz", ref Global.Settings.Hertz, 5.0f, 120.0f, "%.0f hz");

                        ImGui.Separator();

                        ImGui.Checkbox("Sleep", ref Global.Settings.EnableSleep);
                        ImGui.Checkbox("Warm Starting", ref Global.Settings.EnableWarmStarting);
                        ImGui.Checkbox("Time of Impact", ref Global.Settings.EnableContinuous);
                        ImGui.Checkbox("Sub-Stepping", ref Global.Settings.EnableSubStepping);

                        ImGui.Separator();

                        ImGui.Checkbox("Shapes", ref Global.Settings.DrawShapes);
                        ImGui.Checkbox("Joints", ref Global.Settings.DrawJoints);
                        ImGui.Checkbox("AABBs", ref Global.Settings.DrawAABBs);
                        ImGui.Checkbox("Contact Points", ref Global.Settings.DrawContactPoints);
                        ImGui.Checkbox("Contact Normals", ref Global.Settings.DrawContactNormals);
                        ImGui.Checkbox("Contact Impulses", ref Global.Settings.DrawContactImpulse);
                        ImGui.Checkbox("Friction Impulses", ref Global.Settings.DrawFrictionImpulse);
                        ImGui.Checkbox("Center of Masses", ref Global.Settings.DrawCOMs);
                        ImGui.Checkbox("Statistics", ref Global.Settings.DrawStats);
                        ImGui.Checkbox("Profile", ref Global.Settings.DrawProfile);

                        var buttonSz = new System.Numerics.Vector2(-1, 0);
                        if (ImGui.Button("Pause (P)", buttonSz))
                        {
                            Global.Settings.Pause = !Global.Settings.Pause;
                        }

                        if (ImGui.Button("Single Step (O)", buttonSz))
                        {
                            Global.Settings.SingleStep = !Global.Settings.SingleStep;
                        }

                        if (ImGui.Button("Restart (R)", buttonSz))
                        {
                            RestartTest();
                        }

                        if (ImGui.Button("Quit", buttonSz))
                        {
                            Close();
                        }

                        ImGui.EndTabItem();
                    }

                    var leafNodeFlags = ImGuiTreeNodeFlags.OpenOnArrow | ImGuiTreeNodeFlags.OpenOnDoubleClick;
                    leafNodeFlags |= ImGuiTreeNodeFlags.Leaf | ImGuiTreeNodeFlags.NoTreePushOnOpen;

                    const ImGuiTreeNodeFlags NodeFlags = ImGuiTreeNodeFlags.OpenOnArrow | ImGuiTreeNodeFlags.OpenOnDoubleClick;

                    if (ImGui.BeginTabItem("Tests"))
                    {
                        var categoryIndex = 0;
                        var category = Global.Tests[categoryIndex].Category;
                        var i = 0;
                        while (i < Global.Tests.Count)
                        {
                            var categorySelected = string.CompareOrdinal(category, Global.Tests[_currentTestIndex].Category) == 0;
                            var nodeSelectionFlags = categorySelected ? ImGuiTreeNodeFlags.Selected : 0;
                            var nodeOpen = ImGui.TreeNodeEx(category, NodeFlags | nodeSelectionFlags);

                            if (nodeOpen)
                            {
                                while (i < Global.Tests.Count && string.CompareOrdinal(category, Global.Tests[i].Category) == 0)
                                {
                                    ImGuiTreeNodeFlags selectionFlags = 0;
                                    if (_currentTestIndex == i)
                                    {
                                        selectionFlags = ImGuiTreeNodeFlags.Selected;
                                    }

                                    ImGui.TreeNodeEx((IntPtr)i, leafNodeFlags | selectionFlags, Global.Tests[i].Name);
                                    if (ImGui.IsItemClicked())
                                    {
                                        SetTest(i);
                                    }

                                    ++i;
                                }

                                ImGui.TreePop();
                            }
                            else
                            {
                                while (i < Global.Tests.Count && string.CompareOrdinal(category, Global.Tests[i].Category) == 0)
                                {
                                    ++i;
                                }
                            }

                            if (i < Global.Tests.Count)
                            {
                                category = Global.Tests[i].Category;
                                categoryIndex = i;
                            }
                        }

                        ImGui.EndTabItem();
                    }

                    ImGui.EndTabBar();
                }

                ImGui.End();
            }
        }

        /// <inheritdoc />
        protected override void OnClosed()
        {
            Test?.Dispose();
            Test = null;
            Global.DebugDraw.Destroy();
            Global.Settings.Save();
            _controller.Dispose();
            _controller = null;
            _stopwatch.Stop();
            base.OnClosed();
        }

        /// <inheritdoc />
        protected override void OnKeyDown(KeyboardKeyEventArgs e)
        {
            switch (e.Key)
            {
            case Key.Left:
                if (e.Control)
                {
                    Test.ShiftOrigin(new System.Numerics.Vector2(2.0f, 0.0f));
                }
                else
                {
                    Global.Camera.Center.X -= 0.5f;
                }

                break;
            case Key.Right:
                if (e.Control)
                {
                    var newOrigin = new System.Numerics.Vector2(-2.0f, 0.0f);
                    Test.ShiftOrigin(newOrigin);
                }
                else
                {
                    Global.Camera.Center.X += 0.5f;
                }

                break;
            case Key.Up:
                if (e.Control)
                {
                    var newOrigin = new System.Numerics.Vector2(0.0f, -2.0f);
                    Test.ShiftOrigin(newOrigin);
                }
                else
                {
                    Global.Camera.Center.Y += 0.5f;
                }

                break;
            case Key.Down:
                if (e.Control)
                {
                    var newOrigin = new System.Numerics.Vector2(0.0f, 2.0f);
                    Test.ShiftOrigin(newOrigin);
                }
                else
                {
                    Global.Camera.Center.Y -= 0.5f;
                }

                break;
            case Key.Home:
                // Reset view
                Global.Camera.Zoom = 1.0f;
                Global.Camera.Center.Set(0.0f, 20.0f);
                break;
            case Key.Z:
                // Zoom out
                Global.Camera.Zoom = Math.Min(1.1f * Global.Camera.Zoom, 20.0f);
                break;
            case Key.X:
                // Zoom in
                Global.Camera.Zoom = Math.Max(0.9f * Global.Camera.Zoom, 0.02f);
                break;
            case Key.R:
                // Reset test
                RestartTest();
                break;
            case Key.Space:
                // Launch a bomb.
                Test?.LaunchBomb();
                break;
            case Key.O:
                Global.Settings.SingleStep = true;
                break;

            case Key.P:
                Global.Settings.Pause = !Global.Settings.Pause;
                break;

            case Key.BracketLeft:
                // Switch to previous test
                --_testSelection;
                if (_testSelection < 0)
                {
                    _testSelection = Global.Tests.Count - 1;
                }

                break;

            case Key.BracketRight:
                // Switch to next test
                ++_testSelection;
                if (_testSelection == Global.Tests.Count)
                {
                    _testSelection = 0;
                }

                break;

            case Key.Tab:
                Global.DebugDraw.ShowUI = !Global.DebugDraw.ShowUI;
                break;
            default:
                Test?.OnKeyDown(e);
                break;
            }

            base.OnKeyDown(e);
        }

        /// <inheritdoc />
        protected override void OnKeyUp(KeyboardKeyEventArgs e)
        {
            Test.OnKeyUp(e);
            base.OnKeyUp(e);
        }

        /// <inheritdoc />
        protected override void OnMouseDown(MouseButtonEventArgs e)
        {
            if (e.Button == MouseButton.Left)
            {
                var pw = Global.Camera.ConvertScreenToWorld(new System.Numerics.Vector2(MousePosition.X, MousePosition.Y));

                if (e.Modifiers == KeyModifiers.Shift)
                {
                    Test.ShiftMouseDown(pw);
                }
                else
                {
                    Test.MouseDown(pw);
                }
            }

            base.OnMouseDown(e);
        }

        /// <inheritdoc />
        protected override void OnMouseUp(MouseButtonEventArgs e)
        {
            if (e.Button == MouseButton.Left)
            {
                var pw = Global.Camera.ConvertScreenToWorld(new System.Numerics.Vector2(MousePosition.X, MousePosition.Y));
                Test.MouseUp(pw);
            }

            base.OnMouseUp(e);
        }

        /// <inheritdoc />
        protected override void OnMouseMove(MouseMoveEventArgs e)
        {
            if (IsMouseButtonDown(MouseButton.Left))
            {
                var pw = Global.Camera.ConvertScreenToWorld(new System.Numerics.Vector2(MousePosition.X, MousePosition.Y));
                Test.MouseMove(pw);
            }

            if (IsMouseButtonDown(MouseButton.Right))
            {
                Global.Camera.Center.X += e.DeltaX * 0.05f * Global.Camera.Zoom;
                Global.Camera.Center.Y -= e.DeltaY * 0.05f * Global.Camera.Zoom;
            }

            base.OnMouseMove(e);
        }

        public static void ResizeWindowCallback(int width, int height)
        {
            Global.Camera.Width = width;
            Global.Camera.Height = height;
            Global.Settings.WindowWidth = width;
            Global.Settings.WindowHeight = height;
        }

        public static void ScrollCallback(double dx, double dy)
        {
            if (dy > 0)
            {
                Global.Camera.Zoom /= 1.1f;
            }
            else
            {
                Global.Camera.Zoom *= 1.1f;
            }
        }

        #region Test Control

        public void RestartTest()
        {
            Test?.Dispose();
            Test = (Test)Activator.CreateInstance(Global.Tests[_currentTestIndex].TestType);
            if (Test != null)
            {
                Test.Game = this;
            }
        }

        private int _testSelection;

        private static int _currentTestIndex
        {
            get => Global.Settings.TestIndex;
            set => Global.Settings.TestIndex = value;
        }

        private void SetTest(int index)
        {
            _testSelection = index;
        }

        private void CheckTestChange()
        {
            if (_currentTestIndex != _testSelection)
            {
                Test.Dispose();
                _currentTestIndex = _testSelection;
                Test = (Test)Activator.CreateInstance(Global.Tests[_testSelection].TestType);
                if (Test != null)
                {
                    Test.Game = this;
                }
            }
        }

        #endregion

        #region View Control

        public Vector2 Scroll;

        /// <inheritdoc />
        protected override void OnMouseWheel(MouseWheelEventArgs e)
        {
            Scroll = e.Offset;
            ScrollCallback(e.OffsetX, e.OffsetY);
        }

        /// <inheritdoc />
        protected override void OnResize(ResizeEventArgs e)
        {
            GL.Viewport(0, 0, e.Width, e.Height);
            _controller.WindowResized(e.Width, e.Height);
            ResizeWindowCallback(e.Width, e.Height);
            base.OnResize(e);
        }

        #endregion
    }
}