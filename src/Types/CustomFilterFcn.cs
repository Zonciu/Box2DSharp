namespace Box2DSharp
{
    /**@}*/
    /// Prototype for a contact filter callback.
    /// This is called when a contact pair is considered for collision. This allows you to
    ///	perform custom logic to prevent collision between shapes. This is only called if
    ///	one of the two shapes has custom filtering enabled. @see b2ShapeDef.
    /// Notes:
    ///	- this function must be thread-safe
    ///	- this is only called if one of the two shapes has enabled custom filtering
    /// - this is called only for awake dynamic bodies
    ///	Return false if you want to disable the collision
    ///	@warning Do not attempt to modify the world inside this callback
    ///	@ingroup world
    public delegate bool CustomFilterFcn(ShapeId shapeIdA, ShapeId shapeIdB, object context);
}