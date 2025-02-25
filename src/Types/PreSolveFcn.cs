namespace Box2DSharp
{
    /// Prototype for a pre-solve callback.
    /// This is called after a contact is updated. This allows you to inspect a
    /// contact before it goes to the solver. If you are careful, you can modify the
    /// contact manifold (e.g. modify the normal).
    /// Notes:
    ///	- this function must be thread-safe
    ///	- this is only called if the shape has enabled pre-solve events
    /// - this is called only for awake dynamic bodies
    /// - this is not called for sensors
    /// - the supplied manifold has impulse values from the previous step
    ///	Return false if you want to disable the contact this step
    ///	@warning Do not attempt to modify the world inside this callback
    ///	@ingroup world
    public delegate bool PreSolveFcn(ShapeId shapeIdA, ShapeId shapeIdB, ref Manifold manifold, object context);
}