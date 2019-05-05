using System.Collections.Generic;
using UnityEngine;

namespace Box2DSharp.Inspection
{
    [ExecuteInEditMode]
    public abstract class BoundBase : MonoBehaviour
    {
        private const string MainCameraTag = "MainCamera";

        private UnityDrawer _unityDrawer;

        /// <summary>
        /// Line color
        /// </summary>
        public Color Color = Color.green;

        /// <summary>
        ///  Hide Line
        /// </summary>
        public bool Hide;

        /// <summary>
        /// Bound name
        /// </summary>
        public string Name;

        // Update is called once per frame
        private void LateUpdate()
        {
            if (Hide)
            {
                return;
            }

            if (_unityDrawer == default)
            {
                _unityDrawer = UnityDrawer.GetDrawer();
            }

            _unityDrawer.PostLines(DrawLine(), Color);
        }

        protected abstract List<(Vector3 begin, Vector3 end)> DrawLine();
    }
}