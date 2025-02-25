using System;
using ConsoleTest.TestCases;

namespace ConsoleTest
{
    static class Program
    {
        static void Main(string[] args)
        {
            var steps = args.Length > 0 ? int.Parse(args[0]) : 100;
            using var test = new ManyTumbler();
            for (int i = 0; i < steps; i++)
            {
                test.Step();
                Console.WriteLine(i);
            }
        }
    }
}