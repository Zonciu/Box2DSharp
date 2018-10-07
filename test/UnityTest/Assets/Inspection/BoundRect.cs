using System.Collections.Generic;
using UnityEngine;

namespace Box2DSharp.Inspection
{
    public class BoundRect : BoundBase
    {
        public Vector2 BottomLeft;

        public Vector2 BottomRight;

        public Vector2 Max;

        public Vector2 Min;

        public Vector2 TopLeft;

        public Vector2 TopRight;

        public bool TransformBased = true;

        /// <inheritdoc />
        protected override List<(Vector3, Vector3)> DrawLine()
        {
            if (TransformBased)
            {
                var x  = transform.position.x;
                var y  = transform.position.y;
                var hw = transform.localScale.x / 2;
                var hh = transform.localScale.y / 2;

                BottomLeft  = new Vector2(x - hw, y - hh);
                TopLeft     = new Vector2(x - hw, y + hh);
                TopRight    = new Vector2(x + hw, y + hh);
                BottomRight = new Vector2(x + hw, y - hh);
            }
            else
            {
                BottomLeft  = Min;
                TopLeft     = new Vector2(Min.x, Max.y);
                TopRight    = Max;
                BottomRight = new Vector2(Max.x, Min.y);
            }

            var lines = new List<(Vector3, Vector3)>
            {
                (BottomLeft, TopLeft),
                (TopLeft, TopRight),
                (TopRight, BottomRight),
                (BottomRight, BottomLeft)
            };

            // GL.Begin(GL.LINE_STRIP);
            // GL.Color(Color);
            // GL.Vertex(BottomLeft);
            // GL.Vertex(TopLeft);
            // GL.Vertex(TopRight);
            // GL.Vertex(BottomRight);
            // GL.Vertex(BottomLeft);

            // GL.End();
            return lines;
        }
    }
}