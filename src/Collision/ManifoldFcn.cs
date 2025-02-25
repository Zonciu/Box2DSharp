namespace Box2DSharp
{
    public delegate Manifold ManifoldFcn(in Shape shapeA, in Transform xfA, in Shape shapeB, in Transform xfB, ref DistanceCache cache);
}