using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Joints;

/// <summary>
/// This shows how you can implement a constraint outside of Box2D
/// </summary>
[Sample("Joints", "User Constraint")]
public class UserConstraint : SampleBase
{
    public UserConstraint(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (3.0f, -1.0f);
            Global.Camera.Zoom = 25.0f * 0.15f;
        }

        Polygon box = Geometry.MakeBox(1.0f, 0.5f);

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 20.0f;

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;
        bodyDef.GravityScale = 1.0f;
        bodyDef.AngularDamping = 0.5f;
        bodyDef.LinearDamping = 0.2f;
        m_bodyId = Body.CreateBody(WorldId, bodyDef);
        Shape.CreatePolygonShape(m_bodyId, shapeDef, box);

        m_impulses[0] = 0.0f;
        m_impulses[1] = 0.0f;
    }

    protected override void OnRender()
    {
        Transform axes = Transform.Identity;
        Draw.DrawTransform(axes);

        if (Settings.Pause)
        {
            return;
        }

        float timeStep = Settings.Hertz > 0.0f ? 1.0f / Settings.Hertz : 0.0f;
        if (timeStep == 0.0f)
        {
            return;
        }

        float invTimeStep = Settings.Hertz;

        const float hertz = 3.0f;
        const float zeta = 0.7f;
        const float maxForce = 1000.0f;

        float omega = 2.0f * B2Math.Pi * hertz;
        float sigma = 2.0f * zeta + timeStep * omega;
        float s = timeStep * omega * sigma;
        float impulseCoefficient = 1.0f / (1.0f + s);
        float massCoefficient = s * impulseCoefficient;
        float biasCoefficient = omega / sigma;

        Vec2[] localAnchors = [(1.0f, -0.5f), (1.0f, 0.5f)];
        float mass = Body.GetMass(m_bodyId);
        float invMass = mass < 0.0001f ? 0.0f : 1.0f / mass;
        float inertiaTensor = Body.GetRotationalInertia(m_bodyId);
        float invI = inertiaTensor < 0.0001f ? 0.0f : 1.0f / inertiaTensor;

        Vec2 vB = Body.GetLinearVelocity(m_bodyId);
        float omegaB = Body.GetAngularVelocity(m_bodyId);
        Vec2 pB = Body.GetWorldCenterOfMass(m_bodyId);

        for (int i = 0; i < 2; ++i)
        {
            Vec2 anchorA = (3.0f, 0.0f);
            Vec2 anchorB = Body.GetWorldPoint(m_bodyId, localAnchors[i]);

            Vec2 deltaAnchor = anchorB - anchorA;

            float slackLength = 1.0f;
            float length = deltaAnchor.Length;
            float C = length - slackLength;
            if (C < 0.0f || length < 0.001f)
            {
                Draw.DrawSegment(anchorA, anchorB, B2HexColor.LightCyan);
                m_impulses[i] = 0.0f;
                continue;
            }

            Draw.DrawSegment(anchorA, anchorB, B2HexColor.Violet);
            Vec2 axis = deltaAnchor.Normalize;

            Vec2 rB = anchorB - pB;
            float Jb = B2Math.Cross(rB, axis);
            float K = invMass + Jb * invI * Jb;
            float invK = K < 0.0001f ? 0.0f : 1.0f / K;

            float Cdot = B2Math.Dot(vB, axis) + Jb * omegaB;
            float impulse = -massCoefficient * invK * (Cdot + biasCoefficient * C);
            float appliedImpulse = Math.Clamp(impulse, -maxForce * timeStep, 0.0f);

            vB = B2Math.MulAdd(vB, invMass * appliedImpulse, axis);
            omegaB += appliedImpulse * invI * Jb;

            m_impulses[i] = appliedImpulse;
        }

        Body.SetLinearVelocity(m_bodyId, vB);
        Body.SetAngularVelocity(m_bodyId, omegaB);

        DrawString($"forces = {m_impulses[0] * invTimeStep}, {m_impulses[1] * invTimeStep}");
    }

    protected BodyId m_bodyId;

    protected float[] m_impulses = new float[2];
}