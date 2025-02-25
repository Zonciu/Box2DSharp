using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Continuous;


[Sample("Continuous", "Fast Chain")]
public class FastChain : SampleBase
{
    BodyId m_bodyId;

    public FastChain(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 0.0f);
            Global.Camera.Zoom = 25.0f * 0.35f;
        }

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Position = (0.0f, -6.0f);
        BodyId groundId = Body.CreateBody(WorldId, bodyDef);

        Vec2[] points = [(-10.0f, -2.0f), (10.0f, -2.0f), (10.0f, 1.0f), (-10.0f, 1.0f)];

        ChainDef chainDef = ChainDef.DefaultChainDef();
        chainDef.Points = points;
        chainDef.Count = 4;
        chainDef.IsLoop = true;

        Shape.CreateChain(groundId, chainDef);

        m_bodyId = BodyId.NullId;

        Launch();
    }

    protected void Launch()
    {
        if (m_bodyId.IsNotNull)
        {
            Body.DestroyBody(m_bodyId);
        }

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;
        bodyDef.LinearVelocity = (0.0f, -200.0f);
        bodyDef.Position = (0.0f, 10.0f);
        bodyDef.GravityScale = 1.0f;
        m_bodyId = Body.CreateBody(WorldId, bodyDef);

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        Circle circle = ((0.0f, 0.0f), 0.5f);
        Shape.CreateCircleShape(m_bodyId, shapeDef, circle);
    }
}