using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Joints;

[Sample("Joints", "Revolute Joints")]
public class RevoluteJointSample : SampleBase
{
    public RevoluteJointSample(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 15.5f);
            Global.Camera.Zoom = 25.0f * 0.7f;
        }

        BodyId groundId = BodyId.NullId;
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Position = (0.0f, -1.0f);
            groundId = Body.CreateBody(WorldId, bodyDef);

            Polygon box = Geometry.MakeBox(40.0f, 1.0f);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreatePolygonShape(groundId, shapeDef, box);
        }

        m_enableSpring = false;
        m_enableLimit = true;
        m_enableMotor = false;
        m_hertz = 1.0f;
        m_dampingRatio = 0.5f;
        m_motorSpeed = 1.0f;
        m_motorTorque = 1000.0f;

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (-10.0f, 20.0f);
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Density = 1.0f;

            Capsule capsule = ((0.0f, -1.0f), (0.0f, 6.0f), 0.5f);
            Shape.CreateCapsuleShape(bodyId, shapeDef, capsule);

            Vec2 pivot = (-10.0f, 20.5f);
            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
            jointDef.BodyIdA = groundId;
            jointDef.BodyIdB = bodyId;
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.EnableSpring = m_enableSpring;
            jointDef.Hertz = m_hertz;
            jointDef.DampingRatio = m_dampingRatio;
            jointDef.MotorSpeed = m_motorSpeed;
            jointDef.MaxMotorTorque = m_motorTorque;
            jointDef.EnableMotor = m_enableMotor;
            jointDef.ReferenceAngle = 0.5f * B2Math.Pi;
            jointDef.LowerAngle = -0.5f * B2Math.Pi;
            jointDef.UpperAngle = 0.75f * B2Math.Pi;
            jointDef.EnableLimit = m_enableLimit;

            m_jointId1 = Joint.CreateRevoluteJoint(WorldId, jointDef);
        }

        {
            Circle circle = new();
            circle.Radius = 2.0f;

            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (5.0f, 30.0f);
            m_ball = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Density = 1.0f;

            Shape.CreateCircleShape(m_ball, shapeDef, circle);
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Position = (20.0f, 10.0f);
            bodyDef.Type = BodyType.DynamicBody;
            BodyId body = Body.CreateBody(WorldId, bodyDef);

            Polygon box = Geometry.MakeOffsetBox(10.0f, 0.5f, (-10.0f, 0.0f), Rot.Identity);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Density = 1.0f;
            Shape.CreatePolygonShape(body, shapeDef, box);

            Vec2 pivot = (19.0f, 10.0f);
            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
            jointDef.BodyIdA = groundId;
            jointDef.BodyIdB = body;
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.LowerAngle = -0.25f * B2Math.Pi;
            jointDef.UpperAngle = 0.0f * B2Math.Pi;
            jointDef.EnableLimit = true;
            jointDef.EnableMotor = true;
            jointDef.MotorSpeed = 0.0f;
            jointDef.MaxMotorTorque = m_motorTorque;

            m_jointId2 = Joint.CreateRevoluteJoint(WorldId, jointDef);
        }
    }

    protected override void OnRender()
    {
        base.OnRender();

        float angle1 = RevoluteJointFunc.GetAngle(m_jointId1);
        DrawString($"Angle (Deg) 1 = {angle1:00.0}");

        float torque1 = RevoluteJointFunc.GetMotorTorque(m_jointId1);
        DrawString($"Motor Torque 1 = {torque1:0000.0}");

        float torque2 = RevoluteJointFunc.GetMotorTorque(m_jointId2);
        DrawString($"Motor Torque 2 = {torque2:0000.0}");
    }

    protected BodyId m_ball;

    protected JointId m_jointId1;

    protected JointId m_jointId2;

    protected float m_motorSpeed;

    protected float m_motorTorque;

    protected float m_hertz;

    protected float m_dampingRatio;

    protected bool m_enableSpring;

    protected bool m_enableMotor;

    protected bool m_enableLimit;
}