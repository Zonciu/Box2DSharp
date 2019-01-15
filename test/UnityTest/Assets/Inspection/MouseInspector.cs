using UnityEngine;

namespace Box2DSharp.Inspection
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
            ScreenMouse = Input.mousePosition;
            WorldMouse = MainCamera.ScreenToWorldPoint(ScreenMouse);
            ViewPortMouse = MainCamera.ScreenToViewportPoint(ScreenMouse);
        }
    }
}