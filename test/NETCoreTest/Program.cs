using System;
using System.Linq;

namespace NETCoreTest
{
    static class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Tumbler test 2000 step");
            new Test().Tumbler(args.FirstOrDefault() != "1");
        }
    }
}