using System;
using System.Linq;

namespace NETCoreTest
{
    static class Program
    {
        static void Main(string[] args)
        {
            if (!args.Any() || new[] {"/?", "-h", "--help"}.Contains(args[0]))
            {
                Console.WriteLine("arg1: \"1\" stress test, \"0\" normal test");
                Console.WriteLine("arg2: \"1\" show profile");
                return;
            }

            Console.WriteLine("Tumbler test 2000 step");
            var stress = args[0] == "1";
            var showProfile = args.Length >= 2 && args[1] == "1";

            new Test().Tumbler(stress, showProfile);
        }
    }
}