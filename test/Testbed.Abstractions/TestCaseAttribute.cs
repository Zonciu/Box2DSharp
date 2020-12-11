using System;

namespace Testbed.Abstractions
{
    [AttributeUsage(AttributeTargets.Class, AllowMultiple = false, Inherited = true)]
    public class TestCaseAttribute : Attribute
    {
        public TestCaseAttribute(string category, string name)
        {
            Category = category;
            Name = name;
        }

        public string Category { get; }

        public string Name { get; }
    }
}