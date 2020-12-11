using System.Numerics;
using Box2DSharp.Collision;
using Box2DSharp.Common;

namespace Testbed.Abstractions
{
    public interface IDebugDrawer : IDrawer
    {
        void DrawAABB(AABB aabb, Color color);

        void DrawString(float x, float y, string strings);

        void DrawString(int x, int y, string strings);

        void DrawString(Vector2 position, string strings);
    }
}