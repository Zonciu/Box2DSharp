using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;
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

        public Button RestartButton { get; set; }

        public Button QuitButton { get; set; }

        public Toggle ShowToggle { get; set; }

        public GameObject TestObject { get; set; }

        private Type _currentTest;

        private (string TestName, Type TestType)[] _testTypes;

        private void Awake()
        {
            ControlPanel = GameObject.Find("ControlPanel");
            Dropdown = GameObject.Find("TestSelector").GetComponent<Dropdown>()
                    ?? throw new NullReferenceException("TestSelector not found");
            RestartButton = GameObject.Find("RestartButton").GetComponent<Button>()
                         ?? throw new NullReferenceException("RestartButton not found");
            QuitButton = GameObject.Find("QuitButton").GetComponent<Button>()
                      ?? throw new NullReferenceException("QuitButton not found");

            RestartButton.onClick.AddListener(Restart);
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
        }

        private void Start()
        {
            SetTest(_testTypes[0].TestType);
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

        private void SetTest(Type type)
        {
            if (TestObject != null)
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
    }
}