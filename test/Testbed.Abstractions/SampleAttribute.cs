using System;

namespace Testbed.Abstractions
{
    [AttributeUsage(AttributeTargets.Class, AllowMultiple = false, Inherited = true)]
    public class SampleAttribute : Attribute
    {
        public SampleAttribute(string category, string name)
        {
            Category = category;
            Name = name;
        }

        public string Category { get; }

        public string Name { get; }
    }
}