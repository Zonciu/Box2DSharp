using System;
using System.Collections.Generic;
using UnityEngine;

namespace Box2DSharp.Inspection
{
    public class BoundCircle : BoundBase
    {
        public Vector2 Center = Vector2.zero;

        /// <summary>
        /// Circle edge line count, the greater the more accuracy
        /// </summary>
        public int LineCount = 100;

        public float Radius = 1f;

        public bool TransformBased = true;

        public bool TransformCircumcircle = true;

        // Update is called once per frame
        protected override List<(Vector3, Vector3)> DrawLine()
        {
            if (TransformBased)
            {
                Center = transform.position;
            }

            if (TransformCircumcircle)
            {
                Radius = (float) Math.Sqrt(
                             Math.Pow(transform.localScale.x, 2) + Math.Pow(transform.localScale.y, 2))
                       / 2;
            }

            var lines = new List<(Vector3, Vector3)>();

            for (var i = 0; i <= LineCount; ++i) //割圆术画圆
            {
                lines.Add(
                    (
                        new Vector2(
                            Center.x + Radius * Mathf.Cos(2 * Mathf.PI / LineCount * i),
                            Center.y + Radius * Mathf.Sin(2 * Mathf.PI / LineCount * i)),
                        new Vector2(
                            Center.x + Radius * Mathf.Cos(2 * Mathf.PI / LineCount * (i + 1)),
                            Center.y + Radius * Mathf.Sin(2 * Mathf.PI / LineCount * (i + 1)))
                    ));
            }

            return lines;
        }
    }
}