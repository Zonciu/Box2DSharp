using System;

namespace Box2DSharp
{
    /// This struct holds callbacks you can implement to draw a Box2D world.
    ///	This structure should be zero initialized.
    ///	@ingroup world
    public abstract class DebugDrawBase
    {
        /// Bounds to use if restricting drawing to a rectangular region
        public AABB DrawingBounds;

        /// Option to restrict drawing to a rectangular region. May suffer from unstable depth sorting.
        public bool UseDrawingBounds;

        /// Option to draw shapes
        public bool DrawShapes;

        /// Option to draw joints
        public bool DrawJoints;

        /// Option to draw additional information for joints
        public bool DrawJointExtras;

        /// Option to draw the bounding boxes for shapes
        public bool DrawAABBs;

        /// Option to draw the mass and center of mass of dynamic bodies
        public bool DrawMass;

        /// Option to draw contact points
        public bool DrawContacts;

        /// Option to visualize the graph coloring used for contacts and joints
        public bool DrawGraphColors;

        /// Option to draw contact normals
        public bool DrawContactNormals;

        /// Option to draw contact normal impulses
        public bool DrawContactImpulses;

        /// Option to draw contact friction impulses
        public bool DrawFrictionImpulses;

        /// User context that is passed as an argument to drawing callback functions
        public object? Context;

        /// Draw a closed polygon provided in CCW order.
        public abstract void DrawPolygon(ReadOnlySpan<Vec2> vertices, int vertexCount, B2HexColor color, object? context);

        /// Draw a solid closed polygon provided in CCW order.
        public abstract void DrawSolidPolygon(Transform transform, ReadOnlySpan<Vec2> vertices, int vertexCount, float radius, B2HexColor color, object? context);

        /// Draw a circle.
        public abstract void DrawCircle(Vec2 center, float radius, B2HexColor color, object? context);

        /// Draw a solid circle.
        public abstract void DrawSolidCircle(Transform transform, float radius, B2HexColor color, object? context);

        /// Draw a capsule.
        public abstract void DrawCapsule(Vec2 p1, Vec2 p2, float radius, B2HexColor color, object? context);

        /// Draw a solid capsule.
        public abstract void DrawSolidCapsule(Vec2 p1, Vec2 p2, float radius, B2HexColor color, object? context);

        /// Draw a line segment.
        public abstract void DrawSegment(Vec2 p1, Vec2 p2, B2HexColor color, object? context);

        /// Draw a transform. Choose your own length scale.
        public abstract void DrawTransform(Transform transform, object? context);

        /// Draw a point.
        public abstract void DrawPoint(Vec2 p, float size, B2HexColor color, object? context);

        /// Draw a string.
        public abstract void DrawString(Vec2 p, string s, object? context);

        public abstract void DrawBackground();
    }
}