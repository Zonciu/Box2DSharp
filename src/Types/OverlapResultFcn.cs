namespace Box2DSharp
{
    /// Prototype callback for overlap queries.
    /// Called for each shape found in the query.
    /// @see b2World_QueryAABB
    /// @return false to terminate the query.
    ///	@ingroup world
    public delegate bool OverlapResultFcn(ShapeId shapeId, object context);
}