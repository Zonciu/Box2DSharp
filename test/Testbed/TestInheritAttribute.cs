using System;

namespace Testbed
{
    [AttributeUsage(AttributeTargets.Class, AllowMultiple = false, Inherited = false)]
    public class TestInheritAttribute : Attribute
    { }
}