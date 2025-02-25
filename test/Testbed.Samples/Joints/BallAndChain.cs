using System.Diagnostics;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Joints;

[Sample("Joints", "Ball & Chain")]
public class BallAndChain : SampleBase
{
    public const int e_count = 30;

    public BallAndChain(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, -8.0f);
            Global.Camera.Zoom = 27.5f;
        }

        BodyId groundId = BodyId.NullId;
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            groundId = Body.CreateBody(WorldId, bodyDef);
        }

        m_frictionTorque = 100.0f;

        {
            float hx = 0.5f;
            Capsule capsule = ((-hx, 0.0f), (hx, 0.0f), 0.125f);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Density = 20.0f;

            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();

            int jointIndex = 0;

            BodyId prevBodyId = groundId;
            for (int i = 0; i < e_count; ++i)
            {
                BodyDef bodyDef = BodyDef.DefaultBodyDef();
                bodyDef.Type = BodyType.DynamicBody;
                bodyDef.Position = ((1.0f + 2.0f * i) * hx, e_count * hx);
                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
                Shape.CreateCapsuleShape(bodyId, shapeDef, capsule);

                Vec2 pivot = ((2.0f * i) * hx, e_count * hx);
                jointDef.BodyIdA = prevBodyId;
                jointDef.BodyIdB = bodyId;
                jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
                jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);

                // jointDef.EnableMotor = true;
                jointDef.MaxMotorTorque = m_frictionTorque;
                m_jointIds[jointIndex++] = Joint.CreateRevoluteJoint(WorldId, jointDef);

                prevBodyId = bodyId;
            }

            {
                Circle circle = ((0.0f, 0.0f), 4.0f);

                BodyDef bodyDef = BodyDef.DefaultBodyDef();
                bodyDef.Type = BodyType.DynamicBody;
                bodyDef.Position = ((1.0f + 2.0f * e_count) * hx + circle.Radius - hx, e_count * hx);

                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
                Shape.CreateCircleShape(bodyId, shapeDef, circle);

                Vec2 pivot = ((2.0f * e_count) * hx, e_count * hx);
                jointDef.BodyIdA = prevBodyId;
                jointDef.BodyIdB = bodyId;
                jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
                jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
                jointDef.EnableMotor = true;
                jointDef.MaxMotorTorque = m_frictionTorque;
                m_jointIds[jointIndex++] = Joint.CreateRevoluteJoint(WorldId, jointDef);
                Debug.Assert(jointIndex == e_count + 1);
            }
        }
    }

    protected JointId[] m_jointIds = new JointId[e_count + 1];

    protected float m_frictionTorque;
}