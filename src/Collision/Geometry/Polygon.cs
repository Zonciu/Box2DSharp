namespace Box2DSharp
{
    /// <summary>
    /// A solid convex polygon (144 bytes). It is assumed that the interior of the polygon is to
    /// the left of each edge.
    /// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
    /// In most cases you should not need many vertices for a convex polygon.
    ///	@warning DO NOT fill this out manually, instead use a helper function like
    ///	b2MakePolygon or b2MakeBox.
    /// </summary>
    public struct Polygon
    {
        /// <summary>
        /// The polygon vertices
        /// </summary>
        public FixedArray8<Vec2> Vertices;

        /// <summary>
        /// The outward normal vectors of the polygon sides
        /// </summary>
        public FixedArray8<Vec2> Normals;

        /// <summary>
        /// The centroid of the polygon
        /// </summary>
        public Vec2 Centroid;

        /// <summary>
        /// The external radius for rounded polygons
        /// </summary>
        public float Radius;

        /// <summary>
        /// The number of polygon vertices
        /// </summary>
        public int Count;
    }
}