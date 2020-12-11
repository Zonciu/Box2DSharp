using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;

namespace Testbed.Abstractions
{
    public static class Global
    {
        public static IDebugDrawer DebugDrawer { get; set; }

        public static readonly Camera Camera = new Camera();

        public static TestSettings Settings { get; set; }

        public static IInput Input { get; set; }

        public static List<(string Category, string Name, Type TestType)> Tests { get; private set; }

        public static void SetupTestCases(List<Type> testTypes)
        {
            var testBaseType = typeof(TestBase);
            Tests = testTypes
                   .Where(e => testBaseType.IsAssignableFrom(e) && !e.IsAbstract)
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
}