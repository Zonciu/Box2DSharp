using System;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Inspection;
using UnityEditor;
using UnityEngine;
using Logger = Box2DSharp.Common.Logger;
using Vector2 = System.Numerics.Vector2;

namespace Box2DSharp.Tests
{
    public class TestBase : MonoBehaviour
    {
        [HideInInspector]
        public DebugDrawer DebugDrawer;

        [HideInInspector]
        public BoxDrawer WorldDrawer;

        public bool AutoSleep = true;

        public bool DrawShape = true;

        public bool DrawJoint = true;

        public bool DrawAABB = false;

        public bool DrawPair = false;

        public bool DrawCenterOfMass = false;

        public bool DrawContactPoint = false;

        public int Frequency = 60;

        public float InvFrequency;

        public int PositionIteration = 6;

        public int VelocityIteration = 12;

        public World World;

        private void Awake()
        {
            World = new World(new Vector2(0, -10));

            // Define the gravity vector.
            DebugDrawer = DebugDrawer.GetDrawer();

            WorldDrawer = new BoxDrawer {Drawer = DebugDrawer};

            World.SetDebugDrawer(WorldDrawer);
            Logger.SetLogger(new UnityLogger());
        }

        private void Update()
        {
            InvFrequency = 1f / Frequency;
            World.Step(InvFrequency, VelocityIteration, PositionIteration);
        }

        private void LateUpdate()
        {
            DrawFlag flags = 0;
            if (DrawShape)
            {
                flags |= DrawFlag.DrawShape;
            }

            if (DrawJoint)
            {
                flags |= DrawFlag.DrawJoint;
            }

            if (DrawAABB)
            {
                flags |= DrawFlag.DrawAABB;
            }

            if (DrawPair)
            {
                flags |= DrawFlag.DrawPair;
            }

            if (DrawCenterOfMass)
            {
                flags |= DrawFlag.DrawCenterOfMass;
            }

            if (DrawContactPoint)
            {
                flags |= DrawFlag.DrawContactPoint;
            }

            WorldDrawer.Flags = flags;
            World.DrawDebugData();
        }
    }
}