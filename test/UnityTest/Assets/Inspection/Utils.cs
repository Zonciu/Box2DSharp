using UnityEngine;

namespace Box2DSharp.Inspection
{
    public static class Utils
    {
        public static Vector3 ToUnityVector3(this System.Numerics.Vector3 vector3)
        {
            return new Vector3(vector3.X, vector3.Y, vector3.Z);
        }

        public static Vector2 ToUnityVector2(this System.Numerics.Vector3 vector3)
        {
            return new Vector2(vector3.X, vector3.Y);
        }

        public static Vector2 ToUnityVector2(this System.Numerics.Vector2 vector2)
        {
            return new Vector2(vector2.X, vector2.Y);
        }

        public static Vector3 ToUnityVector3(this System.Numerics.Vector2 vector2)
        {
            return new Vector3(vector2.X, vector2.Y, 0);
        }

        public static Color ToUnityColor(this System.Drawing.Color color)
        {
            return new Color(color.R / 255f, color.G / 255f, color.B / 255f, color.A / 255f);
        }
    }
}