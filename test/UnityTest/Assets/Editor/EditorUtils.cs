using System;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEngine;

namespace Box2DSharp.Editor
{
    public class EditorUtils
    {
        [MenuItem("Assets/Box2D/Sort Toggle")]
        public static void SortToggle()
        {
            var toggles = typeof(TestSettings).GetFields()
                                              .Where(f => f.GetCustomAttribute<ToggleAttribute>() != null)
                                              .Select(toggleType => $"ControlPanel/Toggles/{toggleType.Name}")
                                              .Select(
                                                   toggleName =>
                                                       GameObject.Find(toggleName).GetComponent<RectTransform>())
                                              .ToList();

            for (var i = 0; i < toggles.Count; i++)
            {
                var toggle = toggles[i];
                var p = toggle.anchoredPosition;
                p.y = -i * 25;
                toggle.anchoredPosition = p;
            }
        }

        [MenuItem("Assets/Box2D/Show Tests")]
        public static void ShowTests()
        {
            var testNames = typeof(TestBase)
                           .Assembly.GetTypes()
                           .Where(e => e.BaseType == typeof(TestBase))
                           .Select(e => (Regex.Replace(e.Name, @"(\B[A-Z])", " $1")))
                           .ToArray();
            Array.Sort(testNames);
            var sb = new StringBuilder();
            foreach (var testName in testNames)
            {
                sb.AppendLine($"* [x] {testName}");
            }

            Debug.Log(sb.ToString());
        }
    }
}