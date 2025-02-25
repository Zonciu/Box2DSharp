using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Joints;

[Sample("Joints", "Scissor Lift")]
public class ScissorLift : SampleBase
{
    public ScissorLift(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 9.0f);
            Global.Camera.Zoom = 25.0f * 0.4f;
        }

        // Need 8 sub-steps for smoother operation
        settings.SubStepCount = 8;

        BodyId groundId;
        {
            BodyDef bDef = BodyDef.DefaultBodyDef();
            groundId = Body.CreateBody(WorldId, bDef);

            ShapeDef sDef = ShapeDef.DefaultShapeDef();
            Segment segment = ((-20.0f, 0.0f), (20.0f, 0.0f));
            Shape.CreateSegmentShape(groundId, sDef, segment);
        }

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;
        bodyDef.SleepThreshold = 0.01f;

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        Capsule capsule = ((-2.5f, 0.0f), (2.5f, 0.0f), 0.15f);

        BodyId baseId1 = groundId;
        BodyId baseId2 = groundId;
        Vec2 baseAnchor1 = (-2.5f, 0.2f);
        Vec2 baseAnchor2 = (2.5f, 0.2f);
        float y = 0.5f;

        BodyId linkId1 = new();
        int N = 3;

        for (int i = 0; i < N; ++i)
        {
            bodyDef.Position = (0.0f, y);
            bodyDef.Rotation = B2Math.MakeRot(0.15f);
            BodyId bodyId1 = Body.CreateBody(WorldId, bodyDef);
            Shape.CreateCapsuleShape(bodyId1, shapeDef, capsule);

            bodyDef.Position = (0.0f, y);
            bodyDef.Rotation = B2Math.MakeRot(-0.15f);

            BodyId bodyId2 = Body.CreateBody(WorldId, bodyDef);
            Shape.CreateCapsuleShape(bodyId2, shapeDef, capsule);

            if (i == 1)
            {
                linkId1 = bodyId2;
            }

            RevoluteJointDef revoluteDef = RevoluteJointDef.DefaultRevoluteJointDef();

            // left pin
            revoluteDef.BodyIdA = baseId1;
            revoluteDef.BodyIdB = bodyId1;
            revoluteDef.LocalAnchorA = baseAnchor1;
            revoluteDef.LocalAnchorB = (-2.5f, 0.0f);
            revoluteDef.EnableMotor = false;
            revoluteDef.MaxMotorTorque = 1.0f;
            revoluteDef.CollideConnected = i == 0;

            Joint.CreateRevoluteJoint(WorldId, revoluteDef);

            // right pin
            if (i == 0)
            {
                WheelJointDef wheelDef = WheelJointDef.DefaultWheelJointDef();
                wheelDef.BodyIdA = baseId2;
                wheelDef.BodyIdB = bodyId2;
                wheelDef.LocalAxisA = (1.0f, 0.0f);
                wheelDef.LocalAnchorA = baseAnchor2;
                wheelDef.LocalAnchorB = (2.5f, 0.0f);
                wheelDef.EnableSpring = false;
                wheelDef.CollideConnected = true;

                Joint.CreateWheelJoint(WorldId, wheelDef);
            }
            else
            {
                revoluteDef.BodyIdA = baseId2;
                revoluteDef.BodyIdB = bodyId2;
                revoluteDef.LocalAnchorA = baseAnchor2;
                revoluteDef.LocalAnchorB = (2.5f, 0.0f);
                revoluteDef.EnableMotor = false;
                revoluteDef.MaxMotorTorque = 1.0f;
                revoluteDef.CollideConnected = false;

                Joint.CreateRevoluteJoint(WorldId, revoluteDef);
            }

            // middle pin
            revoluteDef.BodyIdA = bodyId1;
            revoluteDef.BodyIdB = bodyId2;
            revoluteDef.LocalAnchorA = (0.0f, 0.0f);
            revoluteDef.LocalAnchorB = (0.0f, 0.0f);
            revoluteDef.EnableMotor = false;
            revoluteDef.MaxMotorTorque = 1.0f;
            revoluteDef.CollideConnected = false;

            Joint.CreateRevoluteJoint(WorldId, revoluteDef);

            baseId1 = bodyId2;
            baseId2 = bodyId1;
            baseAnchor1 = (-2.5f, 0.0f);
            baseAnchor2 = (2.5f, 0.0f);
            y += 1.0f;
        }

        {
            bodyDef.Position = (0.0f, y);
            bodyDef.Rotation = Rot.Identity;
            BodyId platformId = Body.CreateBody(WorldId, bodyDef);

            Polygon box = Geometry.MakeBox(3.0f, 0.2f);
            Shape.CreatePolygonShape(platformId, shapeDef, box);

            // left pin
            RevoluteJointDef revoluteDef = RevoluteJointDef.DefaultRevoluteJointDef();
            revoluteDef.BodyIdA = platformId;
            revoluteDef.BodyIdB = baseId1;
            revoluteDef.LocalAnchorA = (-2.5f, -0.4f);
            revoluteDef.LocalAnchorB = baseAnchor1;
            revoluteDef.EnableMotor = false;
            revoluteDef.MaxMotorTorque = 1.0f;
            revoluteDef.CollideConnected = true;
            Joint.CreateRevoluteJoint(WorldId, revoluteDef);

            // right pin
            WheelJointDef wheelDef = WheelJointDef.DefaultWheelJointDef();
            wheelDef.BodyIdA = platformId;
            wheelDef.BodyIdB = baseId2;
            wheelDef.LocalAxisA = (1.0f, 0.0f);
            wheelDef.LocalAnchorA = (2.5f, -0.4f);
            wheelDef.LocalAnchorB = baseAnchor2;
            wheelDef.EnableSpring = false;
            wheelDef.CollideConnected = true;
            Joint.CreateWheelJoint(WorldId, wheelDef);

            m_enableMotor = false;
            m_motorSpeed = 0.25f;
            m_motorForce = 2000.0f;

            DistanceJointDef distanceDef = DistanceJointDef.DefaultDistanceJointDef();
            distanceDef.BodyIdA = groundId;
            distanceDef.BodyIdB = linkId1;
            distanceDef.LocalAnchorA = (-2.5f, 0.2f);
            distanceDef.LocalAnchorB = (0.5f, 0.0f);
            distanceDef.EnableSpring = true;
            distanceDef.MinLength = 0.2f;
            distanceDef.MaxLength = 5.5f;
            distanceDef.EnableLimit = true;
            distanceDef.EnableMotor = m_enableMotor;
            distanceDef.MotorSpeed = m_motorSpeed;
            distanceDef.MaxMotorForce = m_motorForce;
            m_liftJointId = Joint.CreateDistanceJoint(WorldId, distanceDef);
        }

        Car car = new();
        car.Spawn(WorldId, (0.0f, y + 2.0f), 1.0f, 3.0f, 0.7f, 0.0f, null);
    }

    protected JointId m_liftJointId;

    protected float m_motorForce;

    protected float m_motorSpeed;

    protected bool m_enableMotor;
}