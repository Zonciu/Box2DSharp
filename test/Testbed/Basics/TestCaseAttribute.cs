using System;

namespace Testbed.Basics
{
    [AttributeUsage(AttributeTargets.Class, Inherited = false)]
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