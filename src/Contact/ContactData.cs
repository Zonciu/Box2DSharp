namespace Box2DSharp
{
    /// The contact data for two shapes. By convention the manifold normal points
    ///	from shape A to shape B.
    ///	@see b2Shape_GetContactData() and b2Body_GetContactData()
    public struct ContactData
    {
        public ShapeId ShapeIdA;

        public ShapeId ShapeIdB;

        public Manifold Manifold;

        public ContactData(ShapeId shapeIdA, ShapeId shapeIdB, Manifold manifold)
        {
            ShapeIdA = shapeIdA;
            ShapeIdB = shapeIdB;
            Manifold = manifold;
        }
    }
}