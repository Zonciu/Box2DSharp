using UnityEngine;
using ILogger = Box2DSharp.Common.ILogger;

namespace Box2DSharp
{
    public class UnityLogger : ILogger
    {
        /// <inheritdoc />
        public void Log(string message)
        {
            Debug.Log(message);
        }
    }
}