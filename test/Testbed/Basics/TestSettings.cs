using System;
using System.IO;
using Newtonsoft.Json;
using Newtonsoft.Json.Serialization;

namespace Testbed.Basics
{
    public class TestSettings
    {
        public TestSettings()
        {
            Reset();
        }

        public void Reset()
        {
            TestIndex = 0;
            WindowWidth = 1600;
            WindowHeight = 900;
            Hertz = 60.0f;
            VelocityIterations = 8;
            PositionIterations = 3;
            DrawShapes = true;
            DrawJoints = true;
            DrawAABBs = false;
            DrawContactPoints = false;
            DrawContactNormals = false;
            DrawContactImpulse = false;
            DrawFrictionImpulse = false;
            DrawCOMs = false;
            DrawStats = false;
            DrawProfile = false;
            EnableWarmStarting = true;
            EnableContinuous = true;
            EnableSubStepping = false;
            EnableSleep = true;
            Pause = false;
            SingleStep = false;
            Sleep = true;
            WarmStarting = true;
            TimeOfImpact = true;
            SubStepping = false;
            DrawStats = false;
            DrawProfile = false;
        }

        public void Save()
        {
            var data = new
            {
                TestIndex,
                WindowWidth,
                WindowHeight,
                Hertz,
                VelocityIterations,
                PositionIterations,
                DrawShapes,
                DrawJoints,
                DrawAABBs,
                DrawContactPoints,
                DrawContactNormals,
                DrawContactImpulse,
                DrawFrictionImpulse,
                DrawCOMs,
                DrawStats,
                DrawProfile,
                EnableWarmStarting,
                EnableContinuous,
                EnableSubStepping,
                EnableSleep
            };
            var json = JsonConvert.SerializeObject(
                data,
                new JsonSerializerSettings
                {
                    Formatting = Formatting.Indented,
                    ContractResolver = new CamelCasePropertyNamesContractResolver()
                });
            File.WriteAllText("settings.ini", json);
        }

        public void Load()
        {
            try
            {
                var json = File.ReadAllText("settings.ini");
                JsonConvert.PopulateObject(json, this);
            }
            catch (Exception)
            {
                // ignored
            }
        }

        public bool Sleep;

        public bool WarmStarting;

        public bool TimeOfImpact;

        public bool SubStepping;

        public int TestIndex;

        public int WindowWidth;

        public int WindowHeight;

        public float Hertz;

        public int VelocityIterations;

        public int PositionIterations;

        public bool DrawShapes;

        public bool DrawJoints;

        public bool DrawAABBs;

        public bool DrawContactPoints;

        public bool DrawContactNormals;

        public bool DrawContactImpulse;

        public bool DrawFrictionImpulse;

        public bool DrawCOMs;

        public bool DrawStats;

        public bool DrawProfile;

        public bool EnableWarmStarting;

        public bool EnableContinuous;

        public bool EnableSubStepping;

        public bool EnableSleep;

        public bool Pause;

        public bool SingleStep;
    }
}