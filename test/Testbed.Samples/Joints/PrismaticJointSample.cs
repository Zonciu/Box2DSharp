using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Joints;

[Sample("Joints", "Prismatic Joint")]
public class PrismaticJointSample : SampleBase
{
    public PrismaticJointSample(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 8.0f);
            Global.Camera.Zoom = 25.0f * 0.5f;
        }

        BodyId groundId;
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            groundId = Body.CreateBody(WorldId, bodyDef);
        }

        m_enableSpring = false;
        m_enableLimit = true;
        m_enableMotor = false;
        m_motorSpeed = 2.0f;
        m_motorForce = 25.0f;
        m_hertz = 1.0f;
        m_dampingRatio = 0.5f;

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Position = (0.0f, 10.0f);
            bodyDef.Type = BodyType.DynamicBody;
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Polygon box = Geometry.MakeBox(0.5f, 2.0f);
            Shape.CreatePolygonShape(bodyId, shapeDef, box);

            Vec2 pivot = (0.0f, 9.0f);

            // Vec2 axis = Normalize((1.0f, 0.0f));
            Vec2 axis = Vec2.One.Normalize;
            PrismaticJointDef jointDef = PrismaticJointDef.DefaultPrismaticJointDef();
            jointDef.BodyIdA = groundId;
            jointDef.BodyIdB = bodyId;
            jointDef.LocalAxisA = Body.GetLocalVector(jointDef.BodyIdA, axis);
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.MotorSpeed = m_motorSpeed;
            jointDef.MaxMotorForce = m_motorForce;
            jointDef.EnableMotor = m_enableMotor;
            jointDef.LowerTranslation = -10.0f;
            jointDef.UpperTranslation = 10.0f;
            jointDef.EnableLimit = m_enableLimit;
            jointDef.EnableSpring = m_enableSpring;
            jointDef.Hertz = m_hertz;
            jointDef.DampingRatio = m_dampingRatio;

            m_jointId = Joint.CreatePrismaticJoint(WorldId, jointDef);
        }
    }

    protected override void OnRender()
    {
        float force = PrismaticJointFunc.GetMotorForce(m_jointId);
        DrawString($"Motor Force = {force:0000.1}");
    }

    protected JointId m_jointId;

    protected float m_motorSpeed;

    protected float m_motorForce;

    protected float m_hertz;

    protected float m_dampingRatio;

    protected bool m_enableSpring;

    protected bool m_enableMotor;

    protected bool m_enableLimit;
}