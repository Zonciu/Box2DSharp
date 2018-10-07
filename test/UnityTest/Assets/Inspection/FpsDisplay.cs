using UnityEngine;

namespace Box2DSharp.Inspection
{
    public class FpsDisplay : MonoBehaviour
    {
        private float deltaTime;

        private void Update()
        {
            deltaTime += (Time.unscaledDeltaTime - deltaTime) * 0.1f;
        }

        private void OnGUI()
        {
            int w = Screen.width, h = Screen.height;

            var style = new GUIStyle();

            var rect = new Rect(0, 0, w, h * 2f / 100);
            style.alignment        = TextAnchor.UpperLeft;
            style.fontSize         = h * 2 / 100;
            style.normal.textColor = new Color(0.0f, 0.0f, 0.5f, 1.0f);
            var msec = deltaTime * 1000.0f;
            var fps  = 1.0f / deltaTime;
            var text = $"{msec:0.0} ms ({fps:0.} fps)";
            GUI.Label(rect, text, style);
        }
    }
}