using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Reflection;
using System.Threading;
using Box2DSharp;
using ImGuiNET;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.GraphicsLibraryFramework;
using Testbed.Abstractions;
using Testbed.Gui;
using Testbed.Render;
using Testbed.Samples.Stacking;
using MouseMoveEventArgs = OpenTK.Windowing.Common.MouseMoveEventArgs;
using TKMouseButton = OpenTK.Windowing.GraphicsLibraryFramework.MouseButton;
using Vector2 = System.Numerics.Vector2;

namespace Testbed
{
    public class Game : GameWindow
    {
        private ImGuiController _controller = null!;

        public SampleBase Sample { get; private set; } = null!;

        public readonly GLDraw Draw;

        public readonly Input Input;

        private bool _stopped;

        private readonly Stopwatch _frameCounter = Stopwatch.StartNew();

        /// <inheritdoc />
        public Game(GameWindowSettings gameWindowSettings, NativeWindowSettings nativeWindowSettings)
            : base(gameWindowSettings, nativeWindowSettings)
        {
            Input = new Input(this);
            Global.Input = Input;

            Draw = new GLDraw();
            Global.Draw = Draw;
        }

        public override void Dispose()
        {
            _stopped = true;
            TestSettingHelper.Save(Global.Settings);
            Sample.Dispose();
            Sample = null!;
            Draw.Dispose();
            _controller.Dispose();
            _controller = null!;
            base.Dispose();
        }

        /// <inheritdoc />
        protected override void OnLoad()
        {
            var testBaseType = typeof(SampleBase);
            var testTypes = typeof(SingleBox).Assembly.GetTypes()
                                             .Where(e => testBaseType.IsAssignableFrom(e) && !e.IsAbstract && e.GetCustomAttribute<SampleAttribute>() != null)
                                             .ToHashSet();
            var inheritedTest = this.GetType()
                                    .Assembly.GetTypes()
                                    .Where(
                                         e => testBaseType.IsAssignableFrom(e)
                                           && e.GetCustomAttribute<TestInheritAttribute>() != null
                                           && e.GetCustomAttribute<SampleAttribute>() != null)
                                    .ToList();
            foreach (var type in inheritedTest)
            {
                testTypes.Remove(type.BaseType!);
            }

            var typeList = new List<Type>(testTypes.Count + inheritedTest.Count);
            typeList.AddRange(testTypes);
            typeList.AddRange(inheritedTest);
            Global.SetupSamples(typeList);

            GL.ClearColor(0.2f, 0.2f, 0.2f, 1.0f);
            _controller = new ImGuiController(Size.X, Size.Y);

            _selectedSampleIndex = Math.Clamp(_runningSampleIndex, 0, Global.Samples.Count - 1);
            LoadSample();
            base.OnLoad();
        }

        /// <inheritdoc />
        protected override void OnUnload()
        {
            _stopped = true;
            base.OnUnload();
        }

        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            if (_stopped)
            {
                return;
            }

            var input = KeyboardState;
            if (input.IsKeyDown(Keys.Escape))
            {
                Close();
            }

            if (Global.Settings.Restart)
            {
                LoadSample();
                Global.Settings.Restart = false;
            }

            if (_runningSampleIndex != _selectedSampleIndex)
            {
                LoadSample();
            }

            Sample.PreStep();
            Sample.Step();
            Sample.PostStep();
            base.OnUpdateFrame(e);
        }

        #region Render

        protected override void OnRenderFrame(FrameEventArgs e)
        {
            if (_stopped)
            {
                return;
            }

            _controller.Update(this, (float)e.Time);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
            Draw.DrawBackground();
            UpdateUI();

            Sample.Render();

            var frameTime = (float)_frameCounter.ElapsedTicks / TimeSpan.TicksPerMillisecond;
            _frameCounter.Restart();
            Draw.DrawString(5, Global.Camera.Height - 20, $"{frameTime:00.0} ms - steps: {Sample.StepCount}");

            Draw.Flush();
            _controller.Render();

            Util.CheckGLError("End of frame");
            SwapBuffers();
            base.OnRenderFrame(e);
        }

        public void UpdateUI()
        {
            var maxWorkers = Environment.ProcessorCount;
            const int MenuWidth = 180;
            if (Draw.ShowUI)
            {
                // Draw title
                {
                    ImGui.SetNextWindowPos(new Vector2(0.0f, 0.0f));
                    ImGui.SetNextWindowSize(new Vector2(Global.Camera.Width, Global.Camera.Height));
                    ImGui.SetNextWindowBgAlpha(0);
                    ImGui.Begin("Overlay", ImGuiWindowFlags.NoTitleBar | ImGuiWindowFlags.NoInputs | ImGuiWindowFlags.AlwaysAutoResize | ImGuiWindowFlags.NoScrollbar);
                    ImGui.End();
                    var (category, name, _) = Global.Samples[_runningSampleIndex];
                    Sample.DrawTitle($"{category} : {name}");
                }

                ImGui.SetNextWindowPos(new Vector2((float)Global.Camera.Width - MenuWidth - 10, 10));
                ImGui.SetNextWindowSize(new Vector2(MenuWidth, (float)Global.Camera.Height - 20));

                ImGui.Begin("Tools", ref Draw.ShowUI, ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoCollapse);

                if (ImGui.BeginTabBar("ControlTabs", ImGuiTabBarFlags.None))
                {
                    if (ImGui.BeginTabItem("Controls"))
                    {
                        ImGui.PushItemWidth(100.0f);
                        ImGui.SliderInt("Sub-steps", ref Global.Settings.SubStepCount, 1, 50);
                        ImGui.SliderFloat("Hertz", ref Global.Settings.Hertz, 5.0f, 120.0f, "%.0f hz");

                        if (ImGui.SliderInt("Workers", ref Global.Settings.WorkerCount, 1, maxWorkers))
                        {
                            Global.Settings.WorkerCount = Math.Clamp(Global.Settings.WorkerCount, 1, maxWorkers);
                            RestartSample();
                        }

                        ImGui.PopItemWidth();

                        ImGui.Separator();

                        ImGui.Checkbox("Sleep", ref Global.Settings.EnableSleep);
                        ImGui.Checkbox("Warm Starting", ref Global.Settings.EnableWarmStarting);
                        ImGui.Checkbox("Continuous", ref Global.Settings.EnableContinuous);

                        ImGui.Separator();

                        ImGui.Checkbox("Shapes", ref Global.Settings.DrawShapes);
                        ImGui.Checkbox("Joints", ref Global.Settings.DrawJoints);
                        ImGui.Checkbox("Joint Extras", ref Global.Settings.DrawJointExtras);
                        ImGui.Checkbox("AABBs", ref Global.Settings.DrawAABBs);
                        ImGui.Checkbox("Contact Points", ref Global.Settings.DrawContactPoints);
                        ImGui.Checkbox("Contact Normals", ref Global.Settings.DrawContactNormals);
                        ImGui.Checkbox("Contact Impulses", ref Global.Settings.DrawContactImpulses);
                        ImGui.Checkbox("Friction Impulses", ref Global.Settings.DrawFrictionImpulses);
                        ImGui.Checkbox("Center of Masses", ref Global.Settings.DrawMass);
                        ImGui.Checkbox("Graph Colors", ref Global.Settings.DrawGraphColors);
                        ImGui.Checkbox("Counters", ref Global.Settings.DrawCounters);
                        ImGui.Checkbox("Profile", ref Global.Settings.DrawProfile);

                        var buttonSz = new Vector2(-1, 0);
                        if (ImGui.Button("Pause (P)", buttonSz))
                        {
                            Global.Settings.Pause = !Global.Settings.Pause;
                        }

                        if (ImGui.Button("Single Step (O)", buttonSz))
                        {
                            Global.Settings.SingleStep = !Global.Settings.SingleStep;
                        }

                        if (ImGui.Button("Reset Profile", buttonSz))
                        {
                            Sample.ResetProfile();
                        }

                        if (ImGui.Button("Restart (R)", buttonSz))
                        {
                            RestartSample();
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
                        var category = Global.Samples[categoryIndex].Category;
                        var i = 0;
                        while (i < Global.Samples.Count)
                        {
                            var categorySelected = string.CompareOrdinal(category, Global.Samples[_runningSampleIndex].Category) == 0;
                            var nodeSelectionFlags = categorySelected ? ImGuiTreeNodeFlags.Selected : 0;
                            var nodeOpen = ImGui.TreeNodeEx(category, NodeFlags | nodeSelectionFlags);

                            if (nodeOpen)
                            {
                                while (i < Global.Samples.Count && string.CompareOrdinal(category, Global.Samples[i].Category) == 0)
                                {
                                    ImGuiTreeNodeFlags selectionFlags = 0;
                                    if (_runningSampleIndex == i)
                                    {
                                        selectionFlags = ImGuiTreeNodeFlags.Selected;
                                    }

                                    ImGui.TreeNodeEx((IntPtr)i, leafNodeFlags | selectionFlags, Global.Samples[i].Name);
                                    if (ImGui.IsItemClicked())
                                    {
                                        SetSample(i);
                                    }

                                    ++i;
                                }

                                ImGui.TreePop();
                            }
                            else
                            {
                                while (i < Global.Samples.Count && string.CompareOrdinal(category, Global.Samples[i].Category) == 0)
                                {
                                    ++i;
                                }
                            }

                            if (i < Global.Samples.Count)
                            {
                                category = Global.Samples[i].Category;
                            }
                        }

                        ImGui.EndTabItem();
                    }

                    ImGui.EndTabBar();
                }

                ImGui.End();
                Sample.UpdateUI();
            }
        }

        #endregion

        #region KeyboardControl

        /// <inheritdoc />
        protected override void OnKeyDown(KeyboardKeyEventArgs e)
        {
            switch (e.Key)
            {
            case Keys.Left:
                if (e.Control)
                {
                    Sample.ShiftOrigin(new(2.0f, 0.0f));
                }
                else
                {
                    Global.Camera.Center.X -= 0.5f;
                }

                break;
            case Keys.Right:
                if (e.Control)
                {
                    Sample.ShiftOrigin(new(-2.0f, 0.0f));
                }
                else
                {
                    Global.Camera.Center.X += 0.5f;
                }

                break;
            case Keys.Up:
                if (e.Control)
                {
                    Sample.ShiftOrigin(new(0.0f, -2.0f));
                }
                else
                {
                    Global.Camera.Center.Y += 0.5f;
                }

                break;
            case Keys.Down:
                if (e.Control)
                {
                    Sample.ShiftOrigin(new(0.0f, 2.0f));
                }
                else
                {
                    Global.Camera.Center.Y -= 0.5f;
                }

                break;
            case Keys.Home:
                // Reset view
                Global.Camera.ResetView();
                break;
            case Keys.Z:
                // Zoom out
                Global.Camera.Zoom = Math.Min(1.005f * Global.Camera.Zoom, 100.0f);
                break;
            case Keys.X:
                // Zoom in
                Global.Camera.Zoom = Math.Max(0.995f * Global.Camera.Zoom, 0.5f);
                break;
            case Keys.R:
                // Reset test
                RestartSample();
                break;
            case Keys.O:
                Global.Settings.SingleStep = true;
                break;

            case Keys.P:
                Global.Settings.Pause = !Global.Settings.Pause;
                break;

            case Keys.LeftBracket:
                // Switch to previous test
                --_selectedSampleIndex;
                if (_selectedSampleIndex < 0)
                {
                    _selectedSampleIndex = Global.Samples.Count - 1;
                }

                break;

            case Keys.RightBracket:
                // Switch to next test
                ++_selectedSampleIndex;
                if (_selectedSampleIndex == Global.Samples.Count)
                {
                    _selectedSampleIndex = 0;
                }

                break;

            case Keys.Tab:
                Draw.ShowUI = !Draw.ShowUI;
                break;
            default:
                Sample?.OnKeyDown(new KeyInputEventArgs(Input.GetKeyCode(e.Key), Input.GetKeyModifiers(e.Modifiers), e.IsRepeat));
                break;
            }

            base.OnKeyDown(e);
        }

        /// <inheritdoc />
        protected override void OnKeyUp(KeyboardKeyEventArgs e)
        {
            Sample.OnKeyUp(new KeyInputEventArgs(Input.GetKeyCode(e.Key), Input.GetKeyModifiers(e.Modifiers), e.IsRepeat));
            base.OnKeyUp(e);
        }

        #endregion

        #region MouseControl

        /// <inheritdoc />
        protected override void OnMouseDown(MouseButtonEventArgs e)
        {
            if (ImGui.GetIO().WantCaptureMouse)
            {
                return;
            }

            var ps = new Vec2(MousePosition.X, MousePosition.Y);
            _mouseWorldPosition = Global.Camera.ConvertScreenToWorld(ps);
            if (e.Button == TKMouseButton.Left)
            {
                Sample?.MouseDown(_mouseWorldPosition, e.Convert());
            }

            base.OnMouseDown(e);
        }

        /// <inheritdoc />
        protected override void OnMouseUp(MouseButtonEventArgs e)
        {
            if (ImGui.GetIO().WantCaptureMouse)
            {
                return;
            }

            if (e.Button == TKMouseButton.Left)
            {
                var pw = Global.Camera.ConvertScreenToWorld(new(MousePosition.X, MousePosition.Y));
                Sample?.MouseUp(pw, e.Convert());
            }

            base.OnMouseUp(e);
        }

        private Vec2 _mouseWorldPosition;

        /// <inheritdoc />
        protected override void OnMouseMove(MouseMoveEventArgs e)
        {
            if (ImGui.GetIO().WantCaptureMouse)
            {
                return;
            }

            var ps = new Vec2(MousePosition.X, MousePosition.Y);
            var pw = Global.Camera.ConvertScreenToWorld(ps);
            Sample?.MouseMove(pw, new(e.Position.ToVector2(), e.Delta.ToVector2()));
            if (IsMouseButtonDown(TKMouseButton.Left))
            {
                Sample?.MouseMove(pw, new(e.Position.ToVector2(), e.Delta.ToVector2()));
            }

            if (IsMouseButtonDown(TKMouseButton.Right))
            {
                var diff = pw - _mouseWorldPosition;
                Global.Camera.Center.X -= diff.X;
                Global.Camera.Center.Y -= diff.Y;
                _mouseWorldPosition = Global.Camera.ConvertScreenToWorld(ps);
            }

            base.OnMouseMove(e);
        }

        #endregion

        #region Test Control

        public void RestartSample()
        {
            Global.Settings.Restart = true;
        }

        private int _selectedSampleIndex;

        private static int _runningSampleIndex
        {
            get => Global.Settings.SampleIndex;
            set => Global.Settings.SampleIndex = value;
        }

        private void SetSample(int index)
        {
            _selectedSampleIndex = index;
        }

        private void LoadSample()
        {
            var sample = Sample;
            Sample = null!;
            sample?.Dispose();
            sample = (SampleBase)Activator.CreateInstance(Global.Samples[_selectedSampleIndex].SampleType, Global.Settings)!;
            _runningSampleIndex = _selectedSampleIndex;
            sample.Input = Global.Input;
            sample.Draw = Global.Draw;
            sample.OnInitialized();
            Sample = sample;
        }

        #endregion

        #region View Control

        /// <summary>
        /// IMGUI面板滚动
        /// </summary>
        public Vector2 Scroll;

        /// <inheritdoc />
        protected override void OnMouseWheel(MouseWheelEventArgs e)
        {
            var scroll = MouseState.ScrollDelta.ToVector2();
            if (ImGui.GetIO().WantCaptureMouse)
            {
                Scroll = scroll;
            }
            else
            {
                ScrollCallback(scroll.X, scroll.Y);
            }
        }

        /// <inheritdoc />
        protected override void OnResize(ResizeEventArgs e)
        {
            GL.Viewport(0, 0, e.Width, e.Height);
            _controller.WindowResized(e.Width, e.Height);
            ResizeWindowCallback(e.Width, e.Height);
            base.OnResize(e);
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

        #endregion
    }
}