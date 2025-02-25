using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Continuous;

[Sample("Continuous", "Skinny Box")]
public class SkinnyBox : SampleBase
{
    BodyId m_bodyId;

    BodyId m_bulletId;

    float m_angularVelocity;

    float m_x;

    protected bool m_capsule;

    protected bool m_autoTest;

    bool m_bullet;

    public SkinnyBox(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (1.0f, 5.0f);
            Global.Camera.Zoom = 25.0f * 0.25f;
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            Segment segment = ((-10.0f, 0.0f), (10.0f, 0.0f));
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Friction = 0.9f;
            Shape.CreateSegmentShape(groundId, shapeDef, segment);

            Polygon box = Geometry.MakeOffsetBox(0.1f, 1.0f, (0.0f, 1.0f), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);
        }

        m_autoTest = false;
        m_bullet = false;
        m_capsule = false;

        m_bodyId = BodyId.NullId;
        m_bulletId = BodyId.NullId;

        Launch();
    }

    protected void Launch()
    {
        if (m_bodyId.IsNotNull)
        {
            Body.DestroyBody(m_bodyId);
        }

        if (m_bulletId.IsNotNull)
        {
            Body.DestroyBody(m_bulletId);
        }

        m_angularVelocity = RandomFloat(-50.0f, 50.0f);

        // m_angularVelocity = -30.6695766f;

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;
        bodyDef.Position = (0.0f, 8.0f);
        bodyDef.AngularVelocity = m_angularVelocity;
        bodyDef.LinearVelocity = (0.0f, -100.0f);

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 1.0f;
        shapeDef.Friction = 0.9f;

        m_bodyId = Body.CreateBody(WorldId, bodyDef);

        if (m_capsule)
        {
            Capsule capsule = ((0.0f, -1.0f), (0.0f, 1.0f), 0.1f);
            Shape.CreateCapsuleShape(m_bodyId, shapeDef, capsule);
        }
        else
        {
            Polygon polygon = Geometry.MakeBox(2.0f, 0.05f);
            Shape.CreatePolygonShape(m_bodyId, shapeDef, polygon);
        }

        if (m_bullet)
        {
            Polygon polygon = Geometry.MakeBox(0.25f, 0.25f);
            m_x = RandomFloat(-1.0f, 1.0f);
            bodyDef.Position = (m_x, 10.0f);
            bodyDef.LinearVelocity = (0.0f, -50.0f);
            m_bulletId = Body.CreateBody(WorldId, bodyDef);
            Shape.CreatePolygonShape(m_bulletId, shapeDef, polygon);
        }
    }

    public override void PostStep()
    {
        base.PostStep();
        if (m_autoTest && StepCount % 60 == 0)
        {
            Launch();
        }
    }
}