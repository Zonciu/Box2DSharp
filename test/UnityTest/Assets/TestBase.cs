using System;
using Box2DSharp.Collision;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Box2DSharp.Inspection;
using UnityEngine;
using UnityEngine.Experimental.UIElements;
using Logger = Box2DSharp.Common.Logger;
using Vector2 = System.Numerics.Vector2;

namespace Box2DSharp
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

        public int PositionIteration = 6;

        public int VelocityIteration = 12;

        public World World;

        public Body GroundBody;

        public Camera MainCamera;

        public FrameManager FrameManager;

        private void Awake()
        {
            if (!Camera.main)
            {
                throw new NullReferenceException("Require Main Camera: Camera.main");
            }

            MainCamera = Camera.main;
            World = new World(new Vector2(0, -10));
            GroundBody = World.CreateBody(new BodyDef());
            FrameManager = new FrameManager
            {
                Job = () => { World.Step(1f / Frequency, VelocityIteration, PositionIteration); },
                Interval = 1 / Frequency
            };

            // Define the gravity vector.
            DebugDrawer = DebugDrawer.GetDrawer();
            WorldDrawer = new BoxDrawer {Drawer = DebugDrawer};
            World.SetDebugDrawer(WorldDrawer);
            Logger.SetLogger(new UnityLogger());
        }

        private void Update()
        {
            FrameManager.Tick();

            // Mouse left drag
            Mouse = MainCamera.ScreenToWorldPoint(Input.mousePosition);
            _mouseJoint?.SetTarget(Mouse.ToVector2());
            if (Input.GetMouseButtonDown((int) MouseButton.LeftMouse))
            {
                MouseDown();
            }

            if (Input.GetMouseButtonUp((int) MouseButton.LeftMouse))
            {
                MouseUp();
            }

            // Mouse right move camera
            if (Input.GetMouseButton((int) MouseButton.RightMouse))
            {
                var h = Input.GetAxis("Mouse X");
                var v = Input.GetAxis("Mouse Y");
                MainCamera.transform.Translate(-h, -v, 0, Space.World);
            }

            // Mouse wheel zoom
            //Zoom out
            if (Input.GetAxis("Mouse ScrollWheel") < 0)
            {
                MainCamera.fieldOfView += 2;
                MainCamera.orthographicSize += 1F;
            }

            //Zoom in
            if (Input.GetAxis("Mouse ScrollWheel") > 0)
            {
                MainCamera.fieldOfView -= 2;
                MainCamera.orthographicSize -= 1F;
            }
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

        public Vector3 Mouse;

        private MouseJoint _mouseJoint;

        private void MouseDown()
        {
            if (_mouseJoint != null)
            {
                return;
            }

            var p = Mouse.ToVector2();

            // Make a small box.
            var aabb = new AABB();
            var d = new Vector2(0.001f, 0.001f);
            aabb.LowerBound = p - d;
            aabb.UpperBound = p + d;

            // Query the world for overlapping shapes.
            Fixture fixture = null;
            World.QueryAABB(
                fixture1 =>
                {
                    var body = fixture1.GetBody();
                    if (body.BodyType == BodyType.DynamicBody)
                    {
                        bool inside = fixture1.TestPoint(p);
                        if (inside)
                        {
                            fixture = fixture1;

                            // We are done, terminate the query.
                            return false;
                        }
                    }

                    // Continue the query.
                    return true;
                },
                aabb);

            if (fixture != null)
            {
                var body = fixture.GetBody();
                var md = new MouseJointDef
                {
                    BodyA = GroundBody,
                    BodyB = body,
                    Target = p,
                    MaxForce = 1000.0f * body.Mass
                };
                _mouseJoint = (MouseJoint) World.CreateJoint(md);
                body.IsAwake = true;
            }
        }

        private void MouseUp()
        {
            if (_mouseJoint != null)
            {
                World.DestroyJoint(_mouseJoint);
                _mouseJoint = null;
            }
        }

        private void OnDestroy()
        {
            FrameManager.Dispose();
            World = null;
        }

        /// <summary>
        /// Display Physic Frame
        /// </summary>
        private void OnGUI()
        {
            int w = Screen.width, h = Screen.height;

            var style = new GUIStyle();

            var rect = new Rect(0, h * 2 / 100f, w, h * 2f / 100f);
            style.alignment = TextAnchor.UpperLeft;
            style.fontSize = h * 2 / 100;
            style.normal.textColor = new Color(0.0f, 0.0f, 0.5f, 1.0f);
            var text = $"{FrameManager.FrameCount} Frames";
            GUI.Label(rect, text, style);
        }
    }
}