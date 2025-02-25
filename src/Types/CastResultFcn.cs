namespace Box2DSharp
{
    /// Prototype callback for ray casts.
    /// Called for each shape found in the query. You control how the ray cast
    /// proceeds by returning a float:
    /// return -1: ignore this shape and continue
    /// return 0: terminate the ray cast
    /// return fraction: clip the ray to this point
    /// return 1: don't clip the ray and continue
    /// @param shapeId the shape hit by the ray
    /// @param point the point of initial intersection
    /// @param normal the normal vector at the point of intersection
    /// @param fraction the fraction along the ray at the point of intersection
    ///	@param context the user context
    /// @return -1 to filter, 0 to terminate, fraction to clip the ray for closest hit, 1 to continue
    /// @see b2World_CastRay
    ///	@ingroup world
    public delegate float CastResultFcn(ShapeId shapeId, Vec2 point, Vec2 normal, float fraction, object context);
}