using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;
using Box2DSharp.Inspection;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UI;
using UnityEngine.UIElements;
using Button = UnityEngine.UI.Button;
using Slider = UnityEngine.UI.Slider;
using Toggle = UnityEngine.UI.Toggle;

namespace Box2DSharp
{
    public class TestController : MonoBehaviour
    {
        public GameObject ControlPanel { get; set; }

        public RectTransform Background;

        public Dropdown Dropdown { get; set; }

        public Button PauseButton { get; set; }

        public Button SingleStepButton { get; set; }

        public Button RestartButton { get; set; }

        public Button QuitButton { get; set; }

        public Toggle ShowToggle { get; set; }

        public Slider VelSlider { get; set; }

        public Text VelText { get; set; }

        public Slider PosSlider { get; set; }

        public Text PosText { get; set; }

        public Slider HertzSlider { get; set; }

        public Text HertzText { get; set; }

        public bool MouseInViewPort;

        public bool MouseInUI;

        private int _testIndex = 0;

        [HideInInspector]
        public int CurrentTestIndex = -1;

        [HideInInspector]
        public Test CurrentTest;

        private (string TestName, Type TestType)[] _testTypes;

        public string StartTest;

        public FpsCounter FpsCounter = new FpsCounter();

        public FixedUpdate FixedUpdate;

        private void Awake()
        {
            ControlPanel = GameObject.Find("ControlPanel");
            Background = GameObject.Find("PanelBackground").GetComponent<RectTransform>()
                      ?? throw new NullReferenceException(nameof(Background));
            Dropdown = GameObject.Find("TestSelector").GetComponent<Dropdown>()
                    ?? throw new NullReferenceException("TestSelector not found");
            PauseButton = GameObject.Find("PauseButton").GetComponent<Button>()
                       ?? throw new NullReferenceException("PauseButton not found");
            SingleStepButton = GameObject.Find("SingleStepButton").GetComponent<Button>()
                            ?? throw new NullReferenceException("SingleStepButton not found");
            RestartButton = GameObject.Find("RestartButton").GetComponent<Button>()
                         ?? throw new NullReferenceException("RestartButton not found");
            QuitButton = GameObject.Find("QuitButton").GetComponent<Button>()
                      ?? throw new NullReferenceException("QuitButton not found");

            SingleStepButton.onClick.AddListener(SingleStep);
            RestartButton.onClick.AddListener(Restart);
            PauseButton.onClick.AddListener(Pause);
            QuitButton.onClick.AddListener(Application.Quit);

            _testTypes = typeof(Test).Assembly.GetTypes()
                                     .Where(e => e.BaseType == typeof(Test))
                                     .Select(e => (e.GetCustomAttribute<TestNameAttribute>()?.Name ?? GetTestName(e), e))
                                     .OrderBy(e => e.Item1)
                                     .ToArray();

            Test.TestSettings.UnityDrawer = UnityDrawer.GetDrawer();
            Test.TestSettings.Drawer = new BoxDrawer {Drawer = Test.TestSettings.UnityDrawer};

            Dropdown.ClearOptions();
            Dropdown.AddOptions(_testTypes.OrderBy(e => e.TestName).Select(e => e.TestName).ToList());
            Dropdown.onValueChanged.AddListener(OnTestSelect);

            foreach (var toggleField in typeof(TestSettings)
                                       .GetFields()
                                       .Where(f => f.GetCustomAttribute<ToggleAttribute>() != null))
            {
                var toggleName = $"ControlPanel/Toggles/{toggleField.Name}";
                var toggleObject = GameObject.Find(toggleName)
                                ?? throw new NullReferenceException($"{toggleName} not found");
                var toggle = toggleObject.GetComponent<Toggle>();
                toggle.isOn = (bool) toggleField.GetValue(Test.TestSettings);
                toggle.onValueChanged.AddListener(value => { toggleField.SetValue(Test.TestSettings, value); });
            }

            ShowToggle = GameObject.Find("ShowToggle").GetComponent<Toggle>();
            ShowToggle.onValueChanged.AddListener(ToggleControlPanel);
            Test.TestSettings.ShowControlPanel = ShowToggle.isOn;

            VelSlider = GameObject.Find("VelSlider").GetComponent<Slider>();
            VelText = GameObject.Find("VelText").GetComponent<Text>();
            VelSlider.onValueChanged.AddListener(
                v =>
                {
                    var val = (int) v;
                    VelText.text = GetSliderText(VelText.text, val);
                    Test.TestSettings.VelocityIteration = val;
                });
            VelText.text = GetSliderText(VelText.text, Test.TestSettings.VelocityIteration);

            PosSlider = GameObject.Find("PosSlider").GetComponent<Slider>();
            PosText = GameObject.Find("PosText").GetComponent<Text>();
            PosSlider.onValueChanged.AddListener(
                v =>
                {
                    var val = (int) v;
                    PosText.text = GetSliderText(PosText.text, val);
                    Test.TestSettings.PositionIteration = val;
                });
            PosText.text = GetSliderText(PosText.text, Test.TestSettings.PositionIteration);

            HertzSlider = GameObject.Find("HertzSlider").GetComponent<Slider>();
            HertzText = GameObject.Find("HertzText").GetComponent<Text>();
            HertzSlider.onValueChanged.AddListener(
                v =>
                {
                    var val = (int) v;
                    HertzText.text = GetSliderText(HertzText.text, val);
                    Test.TestSettings.Dt = 1 / (float) val;
                });
            HertzText.text = GetSliderText(HertzText.text, (int) (1 / Test.TestSettings.Dt));

            // DrawString
            _rect = new Rect(20, 20, Screen.width, Screen.height * 2f / 100f);
            _style = new GUIStyle
            {
                alignment = TextAnchor.UpperLeft, fontSize = Screen.height * 2 / 100,
                normal = {textColor = new Color(230f / 255f, 153f / 255f, 153f / 255f, 1.0f)}
            };

            FixedUpdate = new FixedUpdate(TimeSpan.FromSeconds(1 / 60d), Tick);
            MainCamera = Camera.main;
            Test.TestSettings.Camera = MainCamera;
        }

        private void Start()
        {
            var index = Array.FindIndex(_testTypes, t => t.TestName == StartTest);
            Dropdown.value = index;
            SetTest(index);
            FixedUpdate.Start();
        }

        private void Tick()
        {
            CurrentTest.Step();
            FpsCounter.SetFps();
        }

        public Camera MainCamera;

        public Vector3 Diference;

        public Vector3 Origin;

        public bool Drag;

        public void Update()
        {
            if (CurrentTestIndex != _testIndex)
            {
                CurrentTestIndex = _testIndex;
                CurrentTest?.Dispose();
                CurrentTest = (Test) Activator.CreateInstance(_testTypes[CurrentTestIndex].TestType);
                FixedUpdate.Reset();
                FixedUpdate.Start();
            }

            CurrentTest.Update();
            FixedUpdate.Update();
            var mousePosition = Input.mousePosition;
            var rect = Background.rect;
            MouseInViewPort = mousePosition.x > 0
                           && mousePosition.x < Screen.width
                           && mousePosition.y > 0
                           && mousePosition.y < Screen.height;
            MouseInUI = mousePosition.x > Screen.width - rect.width && mousePosition.y > Screen.height - rect.height;
            Test.TestSettings.EnableMouseAction = MouseInViewPort && (!Test.TestSettings.ShowControlPanel || !MouseInUI);

            if (Input.GetKeyDown(KeyCode.R))
            {
                Restart();
            }

            if (Input.GetKeyDown(KeyCode.P))
            {
                Pause();
            }

            if (Input.GetKeyDown(KeyCode.O))
            {
                SingleStep();
            }

            // Launch Bomb
            if (Input.GetKeyDown(KeyCode.Space))
            {
                CurrentTest.LaunchBomb();
            }

            // Mouse left drag
            CurrentTest.MouseWorld = MainCamera.ScreenToWorldPoint(Input.mousePosition).ToVector2();
            CurrentTest.MouseJoint?.SetTarget(CurrentTest.MouseWorld);

            if (Test.TestSettings.EnableMouseAction)
            {
                if (Input.GetMouseButtonDown((int) MouseButton.LeftMouse))
                {
                    if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
                    {
                        CurrentTest.ShiftMouseDown();
                    }
                    else
                    {
                        CurrentTest.MouseDown();
                    }
                }

                if (Input.GetMouseButtonUp((int) MouseButton.LeftMouse))
                {
                    CurrentTest.MouseUp();
                }

                // Mouse right move camera
                if (Input.GetMouseButton((int) MouseButton.RightMouse))
                {
                    Diference = MainCamera.ScreenToWorldPoint(Input.mousePosition)
                              - MainCamera.transform.position;
                    if (Drag == false)
                    {
                        Drag = true;
                        Origin = MainCamera.ScreenToWorldPoint(Input.mousePosition);
                    }
                }
                else
                {
                    Drag = false;
                }

                if (Drag)
                {
                    MainCamera.transform.position = Origin - Diference;
                }

                // Mouse wheel zoom
                //Zoom out
                if (Input.GetAxis("Mouse ScrollWheel") < 0)
                {
                    if (MainCamera.orthographicSize > 1)
                    {
                        MainCamera.orthographicSize += 1f;
                    }
                    else
                    {
                        MainCamera.orthographicSize += 0.1f;
                    }
                }

                //Zoom in
                if (Input.GetAxis("Mouse ScrollWheel") > 0)
                {
                    if (MainCamera.orthographicSize > 1)
                    {
                        MainCamera.orthographicSize -= 1f;
                    }
                    else if (MainCamera.orthographicSize > 0.2f)
                    {
                        MainCamera.orthographicSize -= 0.1f;
                    }
                }
            }

            CurrentTest.DrawString(CurrentTest.TestName);
            if (Test.TestSettings.Pause)
            {
                CurrentTest.DrawString("****PAUSED****");
            }

            // FPS
            {
                var text = $"{FpsCounter.Ms:0.0} ms ({FpsCounter.Fps:F1} fps)";
                CurrentTest.DrawString(text);
            }

            // Step
            {
                CurrentTest.DrawString($"{CurrentTest.StepCount} Steps");
            }
            CurrentTest.DrawTest();
        }

        private Rect _rect;

        private GUIStyle _style;

        private void OnGUI()
        {
            GUI.Label(_rect, CurrentTest.Text, _style);
            CurrentTest.OnGUI();

      
        }

        private void OnTestSelect(int i)
        {
            SetTest(i);
        }

        private string GetTestName(Type type)
        {
            return Regex.Replace(type.Name, @"(\B[A-Z])", " $1");
        }

        private void Restart()
        {
            CurrentTest = (Test) Activator.CreateInstance(CurrentTest.GetType());
        }

        private void Pause()
        {
            Test.TestSettings.Pause = !Test.TestSettings.Pause;
        }

        private void SingleStep()
        {
            Test.TestSettings.SingleStep = true;
        }

        private void SetTest(int index)
        {
            _testIndex = index;
        }

        public void ToggleControlPanel(bool isShow)
        {
            ControlPanel.SetActive(isShow);
            Test.TestSettings.ShowControlPanel = isShow;
        }

        private string GetSliderText(string text, int value)
        {
            return Regex.Replace(text, "([0-9]+)", $"{value}");
        }
    }
}