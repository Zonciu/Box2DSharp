using System.Runtime.Serialization;

namespace Testbed.Abstractions
{
    // todo add camera and Draw and remove globals
    [DataContract]
    public class Settings
    {
        [DataMember]
        public int SampleIndex;

        public int WindowWidth = 1920;

        public int WindowHeight = 1080;

        public float Hertz = 60.0f;

        public int SubStepCount = 4;

        public int WorkerCount = 1;

        [DataMember]
        public bool UseCameraBounds;

        [DataMember]
        public bool DrawShapes = true;

        [DataMember]
        public bool DrawJoints = true;

        [DataMember]
        public bool DrawJointExtras;

        [DataMember]
        public bool DrawAABBs;

        [DataMember]
        public bool DrawContactPoints;

        [DataMember]
        public bool DrawContactNormals;

        [DataMember]
        public bool DrawContactImpulses;

        [DataMember]
        public bool DrawFrictionImpulses;

        [DataMember]
        public bool DrawMass;

        [DataMember]
        public bool DrawGraphColors;

        [DataMember]
        public bool DrawCounters;

        [DataMember]
        public bool DrawProfile;

        [DataMember]
        public bool EnableWarmStarting = true;

        [DataMember]
        public bool EnableContinuous = true;

        [DataMember]
        public bool EnableSleep = true;

        public bool Pause;

        public bool SingleStep;

        public bool Restart;
    }
}