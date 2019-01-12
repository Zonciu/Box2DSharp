using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;
using Box2DSharp.Dynamics;
using Box2DSharp.Inspection;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;

namespace Box2DSharp
{
    public class TestController : MonoBehaviour
    {
        public GameObject ControlPanel { get; set; }

        public TestSettings Settings { get; set; }

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

        public GameObject TestObject { get; set; }

        private Type _currentTest;

        private (string TestName, Type TestType)[] _testTypes;

        private void Awake()
        {
            ControlPanel = GameObject.Find("ControlPanel");
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

            _testTypes = typeof(TestBase).Assembly.GetTypes()
                                         .Where(e => e.BaseType == typeof(TestBase))
                                         .Select(e => (GetTestName(e), e))
                                         .ToArray();

            Settings = gameObject.GetComponent<TestSettings>() ?? gameObject.AddComponent<TestSettings>();
            Settings.DebugDrawer = DebugDrawer.GetDrawer();
            Settings.WorldDrawer = new BoxDrawer {Drawer = Settings.DebugDrawer};

            Dropdown.ClearOptions();
            Dropdown.AddOptions(_testTypes.Select(e => e.TestName).ToList());
            Dropdown.onValueChanged.AddListener(OnTestSelect);

            foreach (var toggleField in typeof(TestSettings)
                                       .GetFields()
                                       .Where(f => f.GetCustomAttribute<ToggleAttribute>() != null))
            {
                var toggleName = $"ControlPanel/Toggles/{toggleField.Name}";
                var toggleObject = GameObject.Find(toggleName)
                                ?? throw new NullReferenceException($"{toggleName} not found");
                var toggle = toggleObject.GetComponent<Toggle>();
                toggle.isOn = (bool) toggleField.GetValue(Settings);
                toggle.onValueChanged.AddListener(value => { toggleField.SetValue(Settings, value); });
            }

            ShowToggle = GameObject.Find("ShowToggle").GetComponent<Toggle>();
            ShowToggle.onValueChanged.AddListener(ToggleControlPanel);

            VelSlider = GameObject.Find("VelSlider").GetComponent<Slider>();
            VelText = GameObject.Find("VelText").GetComponent<Text>();
            VelSlider.onValueChanged.AddListener(
                v =>
                {
                    var val = (int) v;
                    VelText.text = GetSliderText(VelText.text, val);
                    Settings.VelocityIteration = val;
                });
            VelText.text = GetSliderText(VelText.text, Settings.VelocityIteration);

            PosSlider = GameObject.Find("PosSlider").GetComponent<Slider>();
            PosText = GameObject.Find("PosText").GetComponent<Text>();
            PosSlider.onValueChanged.AddListener(
                v =>
                {
                    var val = (int) v;
                    PosText.text = GetSliderText(PosText.text, val);
                    Settings.PositionIteration = val;
                });
            PosText.text = GetSliderText(PosText.text, Settings.PositionIteration);

            HertzSlider = GameObject.Find("HertzSlider").GetComponent<Slider>();
            HertzText = GameObject.Find("HertzText").GetComponent<Text>();
            HertzSlider.onValueChanged.AddListener(
                v =>
                {
                    var val = (int) v;
                    HertzText.text = GetSliderText(HertzText.text, val);
                    Settings.Frequency = val;
                });
            HertzText.text = GetSliderText(HertzText.text, Settings.Frequency);
        }

        private void Start()
        {
            SetTest(_testTypes[0].TestType);
        }

        private void Update()
        {
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
        }

        private void OnTestSelect(int i)
        {
            var test = _testTypes[i];
            Debug.Log($"Select {test.TestName} Test");
            SetTest(test.TestType);
        }

        private string GetTestName(Type type)
        {
            return Regex.Replace(type.Name, @"(\B[A-Z])", " $1");
        }

        private void Restart()
        {
            SetTest(_currentTest);
        }

        private void Pause()
        {
            Settings.Pause = !Settings.Pause;
        }

        private void SingleStep()
        {
            Settings.SingleStep = true;
        }

        private void SetTest(Type type)
        {
            if (TestObject)
            {
                var oldTest = TestObject;
                Destroy(oldTest);
            }

            _currentTest = type;
            TestObject = new GameObject(type.Name);
            TestObject.AddComponent(type);
        }

        public void ToggleControlPanel(bool isShow)
        {
            ControlPanel.SetActive(isShow);
        }

        private string GetSliderText(string text, int value)
        {
            return Regex.Replace(text, "([0-9]+)", $"{value}");
        }
    }
}