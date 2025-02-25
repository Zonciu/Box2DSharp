namespace Box2DSharp
{
    public delegate bool TreeQueryCallbackFcn(int proxyId, int userData, object context);

    public delegate float TreeRayCastCallbackFcn(ref RayCastInput input, int proxyId, int userData, object context);

    public delegate float TreeShapeCastCallbackFcn(ref ShapeCastInput input, int proxyId, int userData, object context);
}