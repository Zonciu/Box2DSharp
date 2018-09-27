using System;

namespace Box2DSharp.Common
{
    public static class Logger
    {
        private static ILogger _logger = new InternalLogger();

        public static void SetLogger(ILogger logger)
        {
            _logger = logger;
        }

        public static void Log(string message)
        {
            _logger.Log(message);
        }
    }

    public interface ILogger
    {
        void Log(string message);
    }

    public class InternalLogger : ILogger
    {
        /// <inheritdoc />
        public void Log(string message)
        {
            Console.WriteLine(message);
        }
    }
}