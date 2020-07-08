using System;
using System.Diagnostics;
using System.Linq;

namespace NETCoreTest
{
    static class Program
    {
        static void Main(string[] args)
        {
            if (!args.Any() || new[] {"/?", "-h", "--help"}.Contains(args[0]))
            {
                PrintHelp();
                return;
            }

            if (args.Any())
            {
                switch (args[0])
                {
                case "stress":
                    StressTest(args);
                    break;
                case "normal":
                    NormalTest(args);
                    break;
                default:
                    PrintHelp();
                    break;
                }
            }
        }

        public static void StressTest(string[] args)
        {
            if (args.Length < 2 || !int.TryParse(args[1], out var count))
            {
                count = 1;
            }

            Console.WriteLine($"Tumbler stress test 2000 step + 2000 steps {count} times");
            var sum1 = 0L;
            var sum2 = 0L;

            //var stress = args[0] == "1";
            //var showProfile = args.Length >= 2 && args[1] == "1";
            for (var i = 0; i < count; i++)
            {
                var test = new Tumbler();
                var t1 = Stopwatch.GetTimestamp();
                for (int j = 0; j < 2000; j++)
                {
                    test.Step();
                }

                t1 = Stopwatch.GetTimestamp() - t1;
                var t2 = Stopwatch.GetTimestamp();
                for (int j = 0; j < 2000; j++)
                {
                    test.Step();
                }

                t2 = Stopwatch.GetTimestamp() - t2;
                Console.WriteLine($"{i + 1}: {t1 / 10000f} ms, {t2 / 10000f} ms");
                sum1 += t1;
                sum2 += t2;
            }

            Console.WriteLine($"Average: {sum1 / (10000f * count)} ms, {sum2 / (10000f * count)} ms");
        }

        public static void NormalTest(string[] args)
        {
            var test = new NormalTest();
            Console.CancelKeyPress += (sender, eventArgs) => { test.Stop(); };
            test.Run();
        }

        static void PrintHelp()
        {
            Console.WriteLine(
                @"Stress test: ConsoleTest stress [iteration]
Normal test: ConsoleTest normal
");
        }
    }
}