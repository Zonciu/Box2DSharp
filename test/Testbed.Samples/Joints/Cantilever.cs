using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Joints;

[Sample("Joints", "Cantilever")]
public class Cantilever : SampleBase
{
    public const int e_count = 8;

    public Cantilever(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 0.0f);
            Global.Camera.Zoom = 25.0f * 0.35f;
        }

        BodyId groundId = BodyId.NullId;
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            groundId = Body.CreateBody(WorldId, bodyDef);
        }

        {
            m_linearHertz = 15.0f;
            m_linearDampingRatio = 0.5f;
            m_angularHertz = 5.0f;
            m_angularDampingRatio = 0.5f;
            m_gravityScale = 1.0f;
            m_collideConnected = false;

            float hx = 0.5f;
            Capsule capsule = ((-hx, 0.0f), (hx, 0.0f), 0.125f);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Density = 20.0f;

            WeldJointDef jointDef = WeldJointDef.DefaultWeldJointDef();

            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.IsAwake = false;

            BodyId prevBodyId = groundId;
            for (int i = 0; i < e_count; ++i)
            {
                bodyDef.Position = ((1.0f + 2.0f * i) * hx, 0.0f);
                m_bodyIds[i] = Body.CreateBody(WorldId, bodyDef);
                Shape.CreateCapsuleShape(m_bodyIds[i], shapeDef, capsule);

                Vec2 pivot = ((2.0f * i) * hx, 0.0f);
                jointDef.BodyIdA = prevBodyId;
                jointDef.BodyIdB = m_bodyIds[i];
                jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
                jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
                jointDef.LinearHertz = m_linearHertz;
                jointDef.LinearDampingRatio = m_linearDampingRatio;
                jointDef.AngularHertz = m_angularHertz;
                jointDef.AngularDampingRatio = m_angularDampingRatio;
                jointDef.CollideConnected = m_collideConnected;
                m_jointIds[i] = Joint.CreateWeldJoint(WorldId, jointDef);

                prevBodyId = m_bodyIds[i];
            }

            m_tipId = prevBodyId;
        }
    }

    protected override void OnRender()
    {
        Vec2 tipPosition = Body.GetPosition(m_tipId);
        DrawString($"tip-y = {tipPosition.Y:F2}");
    }

    protected float m_linearHertz;

    protected float m_linearDampingRatio;

    protected float m_angularHertz;

    protected float m_angularDampingRatio;

    protected float m_gravityScale;

    protected BodyId m_tipId;

    protected BodyId[] m_bodyIds = new BodyId[e_count];

    protected JointId[] m_jointIds = new JointId[e_count];

    protected bool m_collideConnected;
}