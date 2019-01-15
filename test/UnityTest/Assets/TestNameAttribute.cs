using System;

namespace Box2DSharp
{
    [AttributeUsage(AttributeTargets.Class, AllowMultiple = false, Inherited = false)]
    public class TestNameAttribute : Attribute
    {
        public readonly string Name;

        public TestNameAttribute(string name)
        {
            Name = name;
        }
    }
}