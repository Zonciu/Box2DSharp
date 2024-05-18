using Box2DSharp.Common;
using UnityEngine;

namespace Box2DSharp.Testbed.Unity
{
    public class UnityLogger : IDumpLogger
    {
        /// <inheritdoc />
        public void Log(string message)
        {
            Debug.Log(message);
        }
    }
}