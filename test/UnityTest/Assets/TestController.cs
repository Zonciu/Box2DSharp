using System;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;

namespace Box2DSharp
{
    public class TestController : MonoBehaviour
    {
        public Dropdown Dropdown;

        public Button RestartButton;

        private GameObject _testObject;

        private Type _currentTest;

        private Type[] _testTypes;

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

            _testTypes = typeof(TestBase).Assembly.GetTypes().Where(e => e.BaseType == typeof(TestBase)).ToArray();
            Debug.Log(_testTypes.Length);

            Dropdown.ClearOptions();
            Dropdown.AddOptions(_testTypes.Select(e => e.Name).ToList());
            Dropdown.onValueChanged.AddListener(OnTestSelect);

            RestartButton.onClick.AddListener(Restart);
        }

        private void Start()
        {
            SetTest(_testTypes[0]);
        }

        private void OnTestSelect(int i)
        {
            var test = _testTypes[i];
            Debug.Log($"Select {test.Name} Test");
            SetTest(test);
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