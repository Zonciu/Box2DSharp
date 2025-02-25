using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;

namespace Testbed.Abstractions
{
    public static class Global
    {
        public static IDraw Draw { get; set; }

        public static readonly Camera Camera = new Camera();

        public static Settings Settings { get; set; }

        public static IInput Input { get; set; }

        public static List<(string Category, string Name, Type SampleType)> Samples { get; private set; }

        public static void SetupSamples(List<Type> testTypes)
        {
            var testBaseType = typeof(SampleBase);
            Samples = testTypes
                     .Where(e => testBaseType.IsAssignableFrom(e) && !e.IsAbstract)
                     .Select(
                          e =>
                          {
                              var sample = e.GetCustomAttribute<SampleAttribute>() ?? throw new NullReferenceException(e.Name);
                              return (sample.Category, sample.Name, e);
                          })
                     .OrderBy(e => e.Category)
                     .ThenBy(e => e.Name)
                     .ToList();
        }
    }
}