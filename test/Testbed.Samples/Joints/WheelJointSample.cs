using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Joints;

[Sample("Joints", "Wheel Joint")]
public class WheelJointSample : SampleBase
{
    public WheelJointSample(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 10.0f);
            Global.Camera.Zoom = 25.0f * 0.15f;
        }

        BodyId groundId;

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            groundId = Body.CreateBody(WorldId, bodyDef);
        }

        m_enableSpring = true;
        m_enableLimit = true;
        m_enableMotor = true;
        m_motorSpeed = 2.0f;
        m_motorTorque = 5.0f;
        m_hertz = 1.0f;
        m_dampingRatio = 0.7f;

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Position = (0.0f, 10.25f);
            bodyDef.Type = BodyType.DynamicBody;
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Capsule capsule = ((0.0f, -0.5f), (0.0f, 0.5f), 0.5f);
            Shape.CreateCapsuleShape(bodyId, shapeDef, capsule);

            Vec2 pivot = (0.0f, 10.0f);
            Vec2 axis = Vec2.One.Normalize;
            WheelJointDef jointDef = WheelJointDef.DefaultWheelJointDef();
            jointDef.BodyIdA = groundId;
            jointDef.BodyIdB = bodyId;
            jointDef.LocalAxisA = Body.GetLocalVector(jointDef.BodyIdA, axis);
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.MotorSpeed = m_motorSpeed;
            jointDef.MaxMotorTorque = m_motorTorque;
            jointDef.EnableMotor = m_enableMotor;
            jointDef.LowerTranslation = -3.0f;
            jointDef.UpperTranslation = 3.0f;
            jointDef.EnableLimit = m_enableLimit;
            jointDef.Hertz = m_hertz;
            jointDef.DampingRatio = m_dampingRatio;

            m_jointId = Joint.CreateWheelJoint(WorldId, jointDef);
        }
    }

    protected override void OnRender()
    {
        base.OnRender();

        float torque = WheelJointFunc.GetMotorTorque(m_jointId);
        DrawString($"Motor Torque = {torque:0000.0}");
    }

    protected JointId m_jointId;

    protected float m_hertz;

    protected float m_dampingRatio;

    protected float m_motorSpeed;

    protected float m_motorTorque;

    protected bool m_enableSpring;

    protected bool m_enableMotor;

    protected bool m_enableLimit;
}