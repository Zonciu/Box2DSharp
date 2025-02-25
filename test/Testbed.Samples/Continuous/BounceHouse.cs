using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Continuous;

[Sample("Continuous", "Bounce House")]
public class BounceHouse : SampleBase
{
    protected HitEvent[] m_hitEvents = new HitEvent[4];

    protected BodyId m_bodyId;

    protected int m_shapeType;

    protected bool m_enableHitEvents;

    // This tests continuous collision robustness and also demonstrates the speed limits imposed
    // by _maxTranslation and _maxRotation.
    protected struct HitEvent
    {
        public Vec2 Point;

        public float Speed;

        public int StepIndex;
    };

    protected static class TestShapeType
    {
        public const int e_circleShape = 0;

        public const int e_capsuleShape = 1;

        public const int e_boxShape = 2;
    };

    public BounceHouse(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 0.0f);
            Global.Camera.Zoom = 25.0f * 0.45f;
        }

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        BodyId groundId = Body.CreateBody(WorldId, bodyDef);

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        {
            Segment segment = ((-10.0f, -10.0f), (10.0f, -10.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }

        {
            Segment segment = ((10.0f, -10.0f), (10.0f, 10.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }

        {
            Segment segment = ((10.0f, 10.0f), (-10.0f, 10.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }

        {
            Segment segment = ((-10.0f, 10.0f), (-10.0f, -10.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }

        m_shapeType = TestShapeType.e_circleShape;
        m_bodyId = BodyId.NullId;
        m_enableHitEvents = true;

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
        bodyDef.LinearVelocity = (10.0f, 20.0f);
        bodyDef.Position = (0.0f, 0.0f);
        bodyDef.GravityScale = 0.0f;

        // Circle shapes centered on the body can spin fast without risk of tunnelling.
        bodyDef.AllowFastRotation = m_shapeType == TestShapeType.e_circleShape;

        m_bodyId = Body.CreateBody(WorldId, bodyDef);

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 1.0f;
        shapeDef.Restitution = 1.2f;
        shapeDef.Friction = 0.3f;
        shapeDef.EnableHitEvents = m_enableHitEvents;

        if (m_shapeType == TestShapeType.e_circleShape)
        {
            Circle circle = ((0.0f, 0.0f), 0.5f);
            Shape.CreateCircleShape(m_bodyId, shapeDef, circle);
        }
        else if (m_shapeType == TestShapeType.e_capsuleShape)
        {
            Capsule capsule = ((-0.5f, 0.0f), (0.5f, 0.0f), 0.25f);
            Shape.CreateCapsuleShape(m_bodyId, shapeDef, capsule);
        }
        else
        {
            float h = 0.1f;
            Polygon box = Geometry.MakeBox(20.0f * h, h);
            Shape.CreatePolygonShape(m_bodyId, shapeDef, box);
        }
    }


    protected override void OnRender()
    {
        base.OnRender();

        ContactEvents events = World.GetContactEvents(WorldId);
        for (int i = 0; i < events.HitCount; ++i)
        {
            ref ContactHitEvent ev = ref events.HitEvents.Span[i];

            ref HitEvent e = ref m_hitEvents[0];
            for (int j = 1; j < 4; ++j)
            {
                if (m_hitEvents[j].StepIndex < e.StepIndex)
                {
                    e = m_hitEvents[j];
                }
            }

            e.Point = ev.Point;
            e.Speed = ev.ApproachSpeed;
            e.StepIndex = StepCount;
        }

        for (int i = 0; i < 4; ++i)
        {
            ref HitEvent e = ref m_hitEvents[i];
            if (e.StepIndex > 0 && StepCount <= e.StepIndex + 30)
            {
                Draw.DrawCircle(e.Point, 0.1f, B2HexColor.OrangeRed);
                Draw.DrawString(e.Point, e.Speed.ToString("F1"));
            }
        }
    }
}