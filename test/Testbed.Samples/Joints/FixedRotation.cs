using System.Diagnostics;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Joints;

[Sample("Joints", "Fixed Rotation")]
public class FixedRotation : SampleBase
{
    public const int e_count = 6;

    public FixedRotation(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 8.0f);
            Global.Camera.Zoom = 25.0f * 0.7f;
        }

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        m_groundId = Body.CreateBody(WorldId, bodyDef);
        m_fixedRotation = true;

        for (int i = 0; i < e_count; ++i)
        {
            m_bodyIds[i] = BodyId.NullId;
            m_jointIds[i] = JointId.NullId;
        }

        CreateScene();
    }

    void CreateScene()
    {
        for (int i = 0; i < e_count; ++i)
        {
            if ((m_jointIds[i]).IsNotNull)
            {
                Joint.DestroyJoint(m_jointIds[i]);
                m_jointIds[i] = JointId.NullId;
            }

            if ((m_bodyIds[i]).IsNotNull)
            {
                Body.DestroyBody(m_bodyIds[i]);
                m_bodyIds[i] = BodyId.NullId;
            }
        }

        Vec2 position = (-12.5f, 10.0f);
        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;
        bodyDef.FixedRotation = m_fixedRotation;

        Polygon box = Geometry.MakeBox(1.0f, 1.0f);

        int index = 0;

        // distance joint
        {
            Debug.Assert(index < e_count);

            bodyDef.Position = position;
            m_bodyIds[index] = Body.CreateBody(WorldId, bodyDef);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreatePolygonShape(m_bodyIds[index], shapeDef, box);

            float length = 2.0f;
            Vec2 pivot1 = (position.X, position.Y + 1.0f + length);
            Vec2 pivot2 = (position.X, position.Y + 1.0f);
            DistanceJointDef jointDef = DistanceJointDef.DefaultDistanceJointDef();
            jointDef.BodyIdA = m_groundId;
            jointDef.BodyIdB = m_bodyIds[index];
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot1);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot2);
            jointDef.Length = length;
            m_jointIds[index] = Joint.CreateDistanceJoint(WorldId, jointDef);
        }

        position.X += 5.0f;
        ++index;

        // motor joint
        {
            Debug.Assert(index < e_count);

            bodyDef.Position = position;
            m_bodyIds[index] = Body.CreateBody(WorldId, bodyDef);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreatePolygonShape(m_bodyIds[index], shapeDef, box);

            MotorJointDef jointDef = MotorJointDef.DefaultMotorJointDef();
            jointDef.BodyIdA = m_groundId;
            jointDef.BodyIdB = m_bodyIds[index];
            jointDef.LinearOffset = position;
            jointDef.MaxForce = 200.0f;
            jointDef.MaxTorque = 20.0f;
            m_jointIds[index] = Joint.CreateMotorJoint(WorldId, jointDef);
        }

        position.X += 5.0f;
        ++index;

        // prismatic joint
        {
            Debug.Assert(index < e_count);

            bodyDef.Position = position;
            m_bodyIds[index] = Body.CreateBody(WorldId, bodyDef);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreatePolygonShape(m_bodyIds[index], shapeDef, box);

            Vec2 pivot = (position.X - 1.0f, position.Y);
            PrismaticJointDef jointDef = PrismaticJointDef.DefaultPrismaticJointDef();
            jointDef.BodyIdA = m_groundId;
            jointDef.BodyIdB = m_bodyIds[index];
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.LocalAxisA = Body.GetLocalVector(jointDef.BodyIdA, (1.0f, 0.0f));
            m_jointIds[index] = Joint.CreatePrismaticJoint(WorldId, jointDef);
        }

        position.X += 5.0f;
        ++index;

        // revolute joint
        {
            Debug.Assert(index < e_count);

            bodyDef.Position = position;
            m_bodyIds[index] = Body.CreateBody(WorldId, bodyDef);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreatePolygonShape(m_bodyIds[index], shapeDef, box);

            Vec2 pivot = (position.X - 1.0f, position.Y);
            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
            jointDef.BodyIdA = m_groundId;
            jointDef.BodyIdB = m_bodyIds[index];
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            m_jointIds[index] = Joint.CreateRevoluteJoint(WorldId, jointDef);
        }

        position.X += 5.0f;
        ++index;

        // weld joint
        {
            Debug.Assert(index < e_count);

            bodyDef.Position = position;
            m_bodyIds[index] = Body.CreateBody(WorldId, bodyDef);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreatePolygonShape(m_bodyIds[index], shapeDef, box);

            Vec2 pivot = (position.X - 1.0f, position.Y);
            WeldJointDef jointDef = WeldJointDef.DefaultWeldJointDef();
            jointDef.BodyIdA = m_groundId;
            jointDef.BodyIdB = m_bodyIds[index];
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.AngularHertz = 1.0f;
            jointDef.AngularDampingRatio = 0.5f;
            jointDef.LinearHertz = 1.0f;
            jointDef.LinearDampingRatio = 0.5f;
            m_jointIds[index] = Joint.CreateWeldJoint(WorldId, jointDef);
        }

        position.X += 5.0f;
        ++index;

        // wheel joint
        {
            Debug.Assert(index < e_count);

            bodyDef.Position = position;
            m_bodyIds[index] = Body.CreateBody(WorldId, bodyDef);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreatePolygonShape(m_bodyIds[index], shapeDef, box);

            Vec2 pivot = (position.X - 1.0f, position.Y);
            WheelJointDef jointDef = WheelJointDef.DefaultWheelJointDef();
            jointDef.BodyIdA = m_groundId;
            jointDef.BodyIdB = m_bodyIds[index];
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.LocalAxisA = Body.GetLocalVector(jointDef.BodyIdA, (1.0f, 0.0f));
            jointDef.Hertz = 1.0f;
            jointDef.DampingRatio = 0.7f;
            jointDef.LowerTranslation = -1.0f;
            jointDef.UpperTranslation = 1.0f;
            jointDef.EnableLimit = true;
            jointDef.EnableMotor = true;
            jointDef.MaxMotorTorque = 10.0f;
            jointDef.MotorSpeed = 1.0f;
            m_jointIds[index] = Joint.CreateWheelJoint(WorldId, jointDef);
        }

        position.X += 5.0f;
        ++index;
    }

    protected BodyId m_groundId;

    protected BodyId[] m_bodyIds = new BodyId[e_count];

    protected JointId[] m_jointIds = new JointId[e_count];

    protected bool m_fixedRotation;
}