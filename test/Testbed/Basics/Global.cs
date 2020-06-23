using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;

namespace Testbed.Basics
{
    public static class Global
    {
        public static readonly DebugDraw DebugDraw = new DebugDraw();

        public static readonly Camera Camera = new Camera();

        public static readonly TestSettings Settings = new TestSettings();

        public static readonly List<(string Category, string Name, Type TestType)> Tests
            = typeof(Test).Assembly.GetTypes()
                          .Where(e => e.BaseType == typeof(Test))
                          .Select(
                               e =>
                               {
                                   var testCase = e.GetCustomAttribute<TestCaseAttribute>() ?? throw new NullReferenceException(e.Name);
                                   return (testCase.Category, testCase.Name, e);
                               })
                          .OrderBy(e => e.Category)
                          .ThenBy(e => e.Name)
                          .ToList();
    }
}