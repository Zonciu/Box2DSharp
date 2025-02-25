using System.Diagnostics;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Joints;

[Sample("Joints", "Bridge")]
public class Bridge : SampleBase
{
    public const int e_count = 160;

    public Bridge(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Zoom = 25.0f * 2.5f;
        }

        BodyId groundId = BodyId.NullId;
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            groundId = Body.CreateBody(WorldId, bodyDef);
        }

        {
            Polygon box = Geometry.MakeBox(0.5f, 0.125f);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Density = 20.0f;

            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
            int jointIndex = 0;
            m_frictionTorque = 200.0f;
            m_gravityScale = 1.0f;

            float xbase = -80.0f;

            BodyId prevBodyId = groundId;
            for (int i = 0; i < e_count; ++i)
            {
                BodyDef bodyDef = BodyDef.DefaultBodyDef();
                bodyDef.Type = BodyType.DynamicBody;
                bodyDef.Position = (xbase + 0.5f + 1.0f * i, 20.0f);
                bodyDef.LinearDamping = 0.1f;
                bodyDef.AngularDamping = 0.1f;
                m_bodyIds[i] = Body.CreateBody(WorldId, bodyDef);
                Shape.CreatePolygonShape(m_bodyIds[i], shapeDef, box);

                Vec2 pivot = (xbase + 1.0f * i, 20.0f);
                jointDef.BodyIdA = prevBodyId;
                jointDef.BodyIdB = m_bodyIds[i];
                jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
                jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
                jointDef.EnableMotor = true;
                jointDef.MaxMotorTorque = m_frictionTorque;
                m_jointIds[jointIndex++] = Joint.CreateRevoluteJoint(WorldId, jointDef);

                prevBodyId = m_bodyIds[i];
            }

            {
                Vec2 pivot = (xbase + 1.0f * e_count, 20.0f);
                jointDef.BodyIdA = prevBodyId;
                jointDef.BodyIdB = groundId;
                jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
                jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
                jointDef.EnableMotor = true;
                jointDef.MaxMotorTorque = m_frictionTorque;
                m_jointIds[jointIndex++] = Joint.CreateRevoluteJoint(WorldId, jointDef);

                Debug.Assert(jointIndex == e_count + 1);
            }
        }

        for (int i = 0; i < 2; ++i)
        {
            Vec2[] vertices = [(-0.5f, 0.0f), (0.5f, 0.0f), (0.0f, 1.5f)];

            Hull hull = HullFunc.ComputeHull(vertices, 3);
            Polygon triangle = Geometry.MakePolygon(hull, 0.0f);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Density = 20.0f;

            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (-8.0f + 8.0f * i, 22.0f);
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
            Shape.CreatePolygonShape(bodyId, shapeDef, triangle);
        }

        for (int i = 0; i < 3; ++i)
        {
            Circle circle = ((0.0f, 0.0f), 0.5f);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Density = 20.0f;

            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (-6.0f + 6.0f * i, 25.0f);
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
            Shape.CreateCircleShape(bodyId, shapeDef, circle);
        }
    }

    protected BodyId[] m_bodyIds = new BodyId[e_count];

    protected JointId[] m_jointIds = new JointId[e_count + 1];

    protected float m_frictionTorque;

    protected float m_gravityScale;
}