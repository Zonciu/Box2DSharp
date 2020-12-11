using UnityEngine;
using UnityEngine.InputSystem;

namespace Box2DSharp.Testbed.Unity.Inspection
{
    public class MouseInspector : MonoBehaviour
    {
        public Vector2 WorldMouse;

        public Vector2 ScreenMouse;

        public Vector2 ViewPortMouse;

        public Camera MainCamera;

        private void Start()
        {
            MainCamera = Camera.main;
        }

        private void Update()
        {
            ScreenMouse = Mouse.current.position.ReadValue();
            WorldMouse = MainCamera.ScreenToWorldPoint(ScreenMouse);
            ViewPortMouse = MainCamera.ScreenToViewportPoint(ScreenMouse);
        }
    }
}