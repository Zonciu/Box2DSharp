using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Joints;

[Sample("Joints", "Ragdoll")]
public class Ragdoll : SampleBase
{
    public Ragdoll(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 3.0f);
            Global.Camera.Zoom = 25.0f * 0.15f;
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Segment segment = ((-20.0f, 0.0f), (20.0f, 0.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }

        m_jointFrictionTorque = 0.05f;
        m_jointHertz = 0.0f;
        m_jointDampingRatio = 0.5f;

        m_human.Spawn(
            WorldId,
            (0.0f, 5.0f),
            1.0f,
            m_jointFrictionTorque,
            m_jointHertz,
            m_jointDampingRatio,
            1,
            null,
            true);
        m_human.ApplyRandomAngularImpulse(10.0f);
    }

    protected Human m_human = new();

    protected float m_jointFrictionTorque;

    protected float m_jointHertz;

    protected float m_jointDampingRatio;
}