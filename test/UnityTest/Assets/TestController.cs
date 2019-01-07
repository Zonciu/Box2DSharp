using System;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;
using Box2DSharp.Inspection;
using UnityEngine;
using UnityEngine.UI;

namespace Box2DSharp
{
    public class TestController : MonoBehaviour
    {
        public TestSettings Settings;

        public Dropdown Dropdown;

        public Button RestartButton;

        private GameObject _testObject;

        private Type _currentTest;

        private (string TestName, Type TestType)[] _testTypes;

        private void Awake()
        {
            if (!Dropdown)
            {
                throw new NullReferenceException("Test dropdown not found");
            }

            if (!RestartButton)
            {
                throw new NullReferenceException("Restart button not found");
            }

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

            RestartButton.onClick.AddListener(Restart);

            foreach (var toggleField in typeof(TestSettings)
                                       .GetFields()
                                       .Where(f => f.GetCustomAttribute<ToggleAttribute>() != null))
            {
                var toggleName = $"Canvas/{toggleField.Name}";
                var toggleObject = GameObject.Find(toggleName)
                                ?? throw new NullReferenceException($"{toggleName} not found");
                var toggle = toggleObject.GetComponent<Toggle>();
                toggle.isOn = (bool) toggleField.GetValue(Settings);
                toggle.onValueChanged.AddListener(
                    value => { toggleField.SetValue(Settings, value); });
            }
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
            if (_testObject != null)
            {
                var oldTest = _testObject;
                Destroy(oldTest);
            }

            _currentTest = type;
            _testObject = new GameObject(type.Name);
            _testObject.AddComponent(type);
        }
    }
}