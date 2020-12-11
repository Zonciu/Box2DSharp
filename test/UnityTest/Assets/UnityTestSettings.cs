using System.Runtime.Serialization;
using Testbed.Abstractions;
using UnityEngine;

namespace Box2DSharp.Testbed.Unity
{
    [DataContract]
    public class UnityTestSettings : TestSettings
    {
        [DataMember]
        public FullScreenMode FullScreenMode;
    }
}