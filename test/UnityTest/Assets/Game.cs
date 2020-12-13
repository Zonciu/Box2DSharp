using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using Box2DSharp.Common;
using Box2DSharp.Testbed.Unity.Inspection;
using ImGuiNET;
using Testbed.Abstractions;
using Testbed.TestCases;
using UnityEngine;
using UnityEngine.InputSystem;
using Camera = UnityEngine.Camera;

namespace Box2DSharp.Testbed.Unity
{
    public class Game : MonoBehaviour
    {
        public TestBase Test { get; private set; }

        public FpsCounter FpsCounter = new FpsCounter();

        public FixedUpdate FixedUpdate;

        public UnityDrawer UnityDrawer;

        public DebugDrawer DebugDrawer;

        public UnityTestSettings Settings;

        public Camera MainCamera;

        public Vector3 Difference;

        public Vector3 Origin;

        public bool Drag;

        public UnityInput UnityInput;

        public GUIController GUIController;

        private void Awake()
        {
            Settings = TestSettingHelper.Load();
            Global.Settings = Settings;
            Global.Camera.Width = Settings.WindowWidth;
            Global.Camera.Height = Settings.WindowHeight;
            Screen.SetResolution(Settings.WindowWidth, Settings.WindowHeight, Settings.FullScreenMode);

            var testBaseType = typeof(TestBase);
            var allTypes = this.GetType()
                               .Assembly.GetTypes();
            var testTypeArray = allTypes.Where(
                                             e => testBaseType.IsAssignableFrom(e)
                                               && !e.IsAbstract
                                               && e.GetCustomAttribute<TestCaseAttribute>() != null)
                                        .ToArray();
            var testTypes = new HashSet<Type>(testTypeArray);
            var inheritedTest = allTypes.Where(
                                             e => testBaseType.IsAssignableFrom(e)
                                               && e.GetCustomAttribute<TestInheritAttribute>() != null
                                               && e.GetCustomAttribute<TestCaseAttribute>() != null)
                                        .ToList();
            foreach (var type in inheritedTest)
            {
                testTypes.Remove(type.BaseType);
            }

            inheritedTest.ForEach(t => testTypes.Add(t));
            Global.SetupTestCases(testTypes.ToList());

            _screenWidth = Screen.width;
            _screenHeight = Screen.height;

            UnityInput = new UnityInput();
            Global.Input = UnityInput;

            UnityDrawer = UnityDrawer.GetDrawer();
            DebugDrawer = new DebugDrawer {Drawer = UnityDrawer};
            Global.DebugDrawer = DebugDrawer;

            GUIController = new GUIController(this);

            Application.quitting += () => TestSettingHelper.Save(Settings);

            FixedUpdate = new FixedUpdate(TimeSpan.FromSeconds(1 / 60d), Tick);
            MainCamera = Camera.main;
        }

        private void Start()
        {
            CurrentTestIndex = Mathf.Clamp(CurrentTestIndex, 0, Global.Tests.Count - 1);
            if (CurrentTestIndex > Global.Tests.Count || CurrentTestIndex < 0)
            {
                CurrentTestIndex = Global.Tests.FindIndex(e => e.TestType == typeof(HelloWorld));
            }

            _testSelected = CurrentTestIndex;
            RestartTest();
            FixedUpdate.Start();
        }

        private void Tick()
        {
            Test.Step();
            FpsCounter.SetFps();
        }

        public void Update()
        {
            CheckTestChange();
            FixedUpdate.Update();
            if (Test.TestSettings.Pause)
            {
                Test.DrawString("****PAUSED****");
            }

            // FPS
            {
                var text = $"{FpsCounter.Ms:0.0} ms ({FpsCounter.Fps:F1} fps)";
                Test.DrawString(text);
            }

            // Step
            {
                Test.DrawString($"{Test.StepCount} Steps");
            }
            CheckZoom();

            CheckResize();
            CheckKeyDown();
            CheckKeyUp();

            CheckMouseDown();
            CheckMouseMove();
            CheckMouseUp();
        }

        private void OnPreRender()
        {
            Test.Render();
        }

        private void OnEnable()
        {
            ImGuiUn.Layout += RenderUI;
        }

        private void OnDisable()
        {
            ImGuiUn.Layout -= RenderUI;
        }

        private void RenderUI()
        {
            GUIController.Render();
        }

        #region Test Control

        public void RestartTest()
        {
            LoadTest(CurrentTestIndex);
        }

        private int _testSelected;

        public static int CurrentTestIndex
        {
            get => Global.Settings.TestIndex;
            set => Global.Settings.TestIndex = value;
        }

        public void SetTest(int index)
        {
            _testSelected = index;
        }

        private void CheckTestChange()
        {
            if (CurrentTestIndex != _testSelected)
            {
                CurrentTestIndex = _testSelected;
                LoadTest(_testSelected);
            }
        }

        public void LoadTest(int index)
        {
            Test?.Dispose();
            Test = (TestBase)Activator.CreateInstance(Global.Tests[index].TestType);
            if (Test != null)
            {
                Test.Input = Global.Input;
                Test.Drawer = Global.DebugDrawer;
                Test.TestSettings = Global.Settings;
                Test.World.Drawer = Global.DebugDrawer;
            }
        }

        #endregion

        #region KeyboardControl

        public void CheckKeyDown()
        {
            var key = Keyboard.current;
            if (key.leftArrowKey.wasPressedThisFrame)
            {
                if (key.ctrlKey.isPressed)
                {
                    Test.ShiftOrigin(new System.Numerics.Vector2(2.0f, 0.0f));
                }
                else
                {
                    Global.Camera.Center.X -= 0.5f;
                }
            }
            else if (key.rightArrowKey.wasPressedThisFrame)
            {
                if (key.ctrlKey.isPressed)
                {
                    var newOrigin = new System.Numerics.Vector2(-2.0f, 0.0f);
                    Test.ShiftOrigin(newOrigin);
                }
                else
                {
                    Global.Camera.Center.X += 0.5f;
                }
            }
            else if (key.upArrowKey.wasPressedThisFrame)
            {
                if (key.ctrlKey.isPressed)
                {
                    var newOrigin = new System.Numerics.Vector2(0.0f, -2.0f);
                    Test.ShiftOrigin(newOrigin);
                }
                else
                {
                    Global.Camera.Center.Y += 0.5f;
                }
            }
            else if (key.downArrowKey.wasPressedThisFrame)
            {
                if (key.ctrlKey.isPressed)
                {
                    var newOrigin = new System.Numerics.Vector2(0.0f, 2.0f);
                    Test.ShiftOrigin(newOrigin);
                }
                else
                {
                    Global.Camera.Center.Y -= 0.5f;
                }
            }
            else if (key.homeKey.wasPressedThisFrame)
            {
                // Reset view
                Global.Camera.Zoom = 1.0f;
                Global.Camera.Center.Set(0.0f, 20.0f);
            }
            else if (key.zKey.wasPressedThisFrame)
            {
                // Zoom out
                Global.Camera.Zoom = Math.Min(1.1f * Global.Camera.Zoom, 20.0f);
            }
            else if (key.xKey.wasPressedThisFrame)
            {
                // Zoom in
                Global.Camera.Zoom = Math.Max(0.9f * Global.Camera.Zoom, 0.02f);
            }
            else if (key.rKey.wasPressedThisFrame)
            {
                // Reset test
                RestartTest();
            }
            else if (key.spaceKey.wasPressedThisFrame)
            {
                // Launch a bomb.
                Test?.LaunchBomb();
            }
            else if (key.oKey.wasPressedThisFrame)
            {
                Global.Settings.SingleStep = true;
            }
            else if (key.pKey.wasPressedThisFrame)
            {
                Global.Settings.Pause = !Global.Settings.Pause;
            }
            else if (key.leftBracketKey.wasPressedThisFrame)
            {
                // Switch to previous test
                --_testSelected;
                if (_testSelected < 0)
                {
                    _testSelected = Global.Tests.Count - 1;
                }
            }
            else if (key.rightBracketKey.wasPressedThisFrame)
            {
                // Switch to next test
                ++_testSelected;
                if (_testSelected == Global.Tests.Count)
                {
                    _testSelected = 0;
                }
            }
            else if (key.tabKey.wasPressedThisFrame)
            {
                DebugDrawer.ShowUI = !DebugDrawer.ShowUI;
            }
            else if (key.escapeKey.wasPressedThisFrame)
            {
                Application.Quit();
            }
            else
            {
                foreach (var keyCode in UnityInput.KeyCodeMap)
                {
                    if (key[keyCode.Value].wasPressedThisFrame)
                    {
                        Test?.OnKeyDown(
                            new KeyInputEventArgs(
                                keyCode.Key,
                                UnityInput.GetKeyModifiers(key),
                                false));
                    }
                }
            }
        }

        private void CheckKeyUp()
        {
            var key = Keyboard.current;
            foreach (var keyCode in UnityInput.KeyCodeMap)
            {
                if (key[keyCode.Value].wasReleasedThisFrame)
                {
                    Test?.OnKeyUp(
                        new KeyInputEventArgs(
                            keyCode.Key,
                            UnityInput.GetKeyModifiers(key),
                            false));
                }
            }
        }

        #endregion

        #region MouseControl

        private void CheckMouseDown()
        {
            var mouse = Mouse.current;
            var mousePosition = Mouse.current.position.ReadValue();
            var keyborar = Keyboard.current;

            if (mouse.leftButton.wasPressedThisFrame)
            {
                var pw = MainCamera.ScreenToWorldPoint(mousePosition).ToVector2();
                if (keyborar.shiftKey.isPressed)
                {
                    // Mouse left drag
                    Test.ShiftMouseDown(pw);
                }
                else
                {
                    Test.MouseDown(pw);
                }
            }

            // Mouse right move camera
            if (mouse.rightButton.isPressed)
            {
                Difference = MainCamera.ScreenToWorldPoint(mousePosition)
                           - MainCamera.transform.position;
                if (Drag == false)
                {
                    Drag = true;
                    Origin = MainCamera.ScreenToWorldPoint(mousePosition);
                }
            }
            else
            {
                Drag = false;
            }
        }

        private void CheckMouseUp()
        {
            if (Mouse.current.leftButton.wasReleasedThisFrame)
            {
                Test.MouseUp(MainCamera.ScreenToWorldPoint(Mouse.current.position.ReadValue()).ToVector2());
            }
        }

        private void CheckMouseMove()
        {
            if (Mouse.current.leftButton.isPressed)
            {
                var mousePosition = Mouse.current.position.ReadValue();
                var pw = MainCamera.ScreenToWorldPoint(new Vector2(mousePosition.x, mousePosition.y)).ToVector2();
                Test.MouseMove(pw);
            }

            if (Mouse.current.rightButton.isPressed)
            {
                var delta = Mouse.current.delta.ReadValue();
                Global.Camera.Center.X -= delta.x * 0.05f * Global.Camera.Zoom;
                Global.Camera.Center.Y += delta.y * 0.05f * Global.Camera.Zoom;
            }

            if (Drag)
            {
                MainCamera.transform.position = Origin - Difference;
            }
        }

        #endregion

        #region View Control

        public Vector2 Scroll;

        /// <summary>
        /// Mouse wheel zoom
        /// </summary>
        private void CheckZoom()
        {
            var scroll = Mouse.current.scroll.ReadValue();

            //Zoom out
            if (scroll.y < 0)
            {
                if (MainCamera.orthographicSize > 1)
                {
                    MainCamera.orthographicSize += 1f;
                }
                else
                {
                    MainCamera.orthographicSize += 0.1f;
                }

                Scroll = scroll;
                ScrollCallback(Scroll.x, Scroll.y);
            }

            //Zoom in
            if (scroll.y > 0)
            {
                if (MainCamera.orthographicSize > 1)
                {
                    MainCamera.orthographicSize -= 1f;
                }
                else if (MainCamera.orthographicSize > 0.2f)
                {
                    MainCamera.orthographicSize -= 0.1f;
                }

                Scroll = scroll;
                ScrollCallback(Scroll.x, Scroll.y);
            }
        }

        private int _screenWidth;

        private int _screenHeight;

        private FullScreenMode _mode;

        private void CheckResize()
        {
            var w = Screen.width;
            var h = Screen.height;
            var mode = Screen.fullScreenMode;
            if (_screenWidth != w || _screenHeight != h || _mode != mode)
            {
                _screenWidth = w;
                _screenHeight = h;
                _mode = mode;
                GL.Viewport(new Rect(0, 0, w, h));
                ResizeWindowCallback(w, h, mode);
            }
        }

        public void ResizeWindowCallback(int width, int height, FullScreenMode fullScreenMode)
        {
            Global.Camera.Width = width;
            Global.Camera.Height = height;
            Settings.WindowWidth = width;
            Settings.WindowHeight = height;
            Settings.FullScreenMode = fullScreenMode;
        }

        public void ScrollCallback(double dx, double dy)
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