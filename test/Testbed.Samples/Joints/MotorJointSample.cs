using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Joints;

/// This test shows how to use a motor joint. A motor joint
/// can be used to animate a dynamic body. With finite motor forces
/// the body can be blocked by collision with other bodies.
///	By setting the correction factor to zero, the motor joint acts
///	like top-down dry friction.
[Sample("Joints", "Motor Joint")]
public class MotorJointSample : SampleBase
{
    public MotorJointSample(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 7.0f);
            Global.Camera.Zoom = 25.0f * 0.4f;
        }

        BodyId groundId;
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            groundId = Body.CreateBody(WorldId, bodyDef);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Segment segment = ((-20.0f, 0.0f), (20.0f, 0.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }

        // Define motorized body
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (0.0f, 8.0f);
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            Polygon box = Geometry.MakeBox(2.0f, 0.5f);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Density = 1.0f;
            Shape.CreatePolygonShape(bodyId, shapeDef, box);

            m_maxForce = 500.0f;
            m_maxTorque = 500.0f;
            m_correctionFactor = 0.3f;

            MotorJointDef jointDef = MotorJointDef.DefaultMotorJointDef();
            jointDef.BodyIdA = groundId;
            jointDef.BodyIdB = bodyId;
            jointDef.MaxForce = m_maxForce;
            jointDef.MaxTorque = m_maxTorque;
            jointDef.CorrectionFactor = m_correctionFactor;

            m_jointId = Joint.CreateMotorJoint(WorldId, jointDef);
        }

        m_go = true;
        m_time = 0.0f;
    }

    protected override void OnRender()
    {
        base.OnRender();

        if (m_go && Settings.Hertz > 0.0f)
        {
            m_time += 1.0f / Settings.Hertz;
        }

        Vec2 linearOffset;
        linearOffset.X = 6.0f * MathF.Sin(2.0f * m_time);
        linearOffset.Y = 8.0f + 4.0f * MathF.Sin(1.0f * m_time);

        float angularOffset = B2Math.Pi * MathF.Sin(-0.5f * m_time);

        MotorJointFunc.SetLinearOffset(m_jointId, linearOffset);
        MotorJointFunc.SetAngularOffset(m_jointId, angularOffset);

        Transform transform = (linearOffset, B2Math.MakeRot(angularOffset));
        Draw.DrawTransform(transform);

        Vec2 force = Joint.GetConstraintForce(m_jointId);
        float torque = Joint.GetConstraintTorque(m_jointId);

        DrawString($"force = ({force.X}, {force.Y}), torque = {torque}");
    }

    protected JointId m_jointId;

    protected float m_time;

    protected float m_maxForce;

    protected float m_maxTorque;

    protected float m_correctionFactor;

    protected bool m_go;
}