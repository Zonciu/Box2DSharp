using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Joints;

/// <summary>
/// Test the distance joint and all options
/// </summary>
[Sample("Joints", "Distance Joint")]
public class DistanceJointSample : SampleBase
{
    public const int e_maxCount = 10;

    public DistanceJointSample(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 12.0f);
            Global.Camera.Zoom = 25.0f * 0.35f;
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            m_groundId = Body.CreateBody(WorldId, bodyDef);
        }

        m_count = 0;
        m_hertz = 2.0f;
        m_dampingRatio = 0.5f;
        m_length = 1.0f;
        m_minLength = m_length;
        m_maxLength = m_length;
        m_enableSpring = false;
        m_enableLimit = false;

        for (int i = 0; i < e_maxCount; ++i)
        {
            m_bodyIds[i] = BodyId.NullId;
            m_jointIds[i] = JointId.NullId;
        }

        CreateScene(1);
    }

    protected void CreateScene(int newCount)
    {
        // Must destroy joints before bodies
        for (int i = 0; i < m_count; ++i)
        {
            Joint.DestroyJoint(m_jointIds[i]);
            m_jointIds[i] = JointId.NullId;
        }

        for (int i = 0; i < m_count; ++i)
        {
            Body.DestroyBody(m_bodyIds[i]);
            m_bodyIds[i] = BodyId.NullId;
        }

        m_count = newCount;

        float radius = 0.25f;
        Circle circle = ((0.0f, 0.0f), radius);

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 20.0f;

        float yOffset = 20.0f;

        DistanceJointDef jointDef = DistanceJointDef.DefaultDistanceJointDef();
        jointDef.Hertz = m_hertz;
        jointDef.DampingRatio = m_dampingRatio;
        jointDef.Length = m_length;
        jointDef.MinLength = m_minLength;
        jointDef.MaxLength = m_maxLength;
        jointDef.EnableSpring = m_enableSpring;
        jointDef.EnableLimit = m_enableLimit;

        BodyId prevBodyId = m_groundId;
        for (int i = 0; i < m_count; ++i)
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (m_length * (i + 1.0f), yOffset);
            m_bodyIds[i] = Body.CreateBody(WorldId, bodyDef);
            Shape.CreateCircleShape(m_bodyIds[i], shapeDef, circle);

            Vec2 pivotA = (m_length * i, yOffset);
            Vec2 pivotB = (m_length * (i + 1.0f), yOffset);
            jointDef.BodyIdA = prevBodyId;
            jointDef.BodyIdB = m_bodyIds[i];
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivotA);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivotB);
            m_jointIds[i] = Joint.CreateDistanceJoint(WorldId, jointDef);

            prevBodyId = m_bodyIds[i];
        }
    }

    protected BodyId m_groundId;

    protected BodyId[] m_bodyIds = new BodyId[e_maxCount];

    protected JointId[] m_jointIds = new JointId[e_maxCount];

    protected int m_count;

    protected float m_hertz;

    protected float m_dampingRatio;

    protected float m_length;

    protected float m_minLength;

    protected float m_maxLength;

    protected bool m_enableSpring;

    protected bool m_enableLimit;
}

// A suspension bridge

// This sample shows the limitations of an iterative solver. The cantilever sags even though the weld
// joint is stiff as possible.

// This test ensures joints work correctly with bodies that have fixed rotation

// This sample shows how to break joints when the internal reaction force becomes large.

// This is a fun demo that shows off the wheel joint