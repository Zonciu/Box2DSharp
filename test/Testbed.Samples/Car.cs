using System.Diagnostics;
using Box2DSharp;

namespace Testbed.Samples;

public class Car
{
    public BodyId ChassisId;

    public BodyId RearWheelId;

    public BodyId FrontWheelId;

    public JointId RearAxleId;

    public JointId FrontAxleId;

    public bool IsSpawned;

    public Car()
    {
        ChassisId = new();
        RearWheelId = new();
        FrontWheelId = new();
        RearAxleId = new();
        FrontAxleId = new();
        IsSpawned = false;
    }

    public void Spawn(WorldId worldId, Vec2 position, float scale, float hertz, float dampingRatio, float torque, object? userData)
    {
        Debug.Assert(IsSpawned == false);

        Debug.Assert(ChassisId.IsNull);
        Debug.Assert(FrontWheelId.IsNull);
        Debug.Assert(RearWheelId.IsNull);

        Vec2[] vertices = [(-1.5f, -0.5f), (1.5f, -0.5f), (1.5f, 0.0f), (0.0f, 0.9f), (-1.15f, 0.9f), (-1.5f, 0.2f),];

        for (int i = 0; i < 6; ++i)
        {
            vertices[i].X *= 0.85f * scale;
            vertices[i].Y *= 0.85f * scale;
        }

        Hull hull = HullFunc.ComputeHull(vertices, 6);
        Polygon chassis = Geometry.MakePolygon(hull, 0.15f * scale);

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 1.0f / scale;
        shapeDef.Friction = 0.2f;

        Circle circle = ((0.0f, 0.0f), 0.4f * scale);

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;
        bodyDef.Position = (0.0f, 1.0f * scale) + position;
        ChassisId = Body.CreateBody(worldId, bodyDef);
        Shape.CreatePolygonShape(ChassisId, shapeDef, chassis);

        shapeDef.Density = 2.0f / scale;
        shapeDef.Friction = 1.5f;

        bodyDef.Position = ((-1.0f * scale, 0.35f * scale) + position);
        bodyDef.AllowFastRotation = true;
        RearWheelId = Body.CreateBody(worldId, bodyDef);
        Shape.CreateCircleShape(RearWheelId, shapeDef, circle);

        bodyDef.Position = ((1.0f * scale, 0.4f * scale) + position);
        bodyDef.AllowFastRotation = true;
        FrontWheelId = Body.CreateBody(worldId, bodyDef);
        Shape.CreateCircleShape(FrontWheelId, shapeDef, circle);

        Vec2 axis = (0.0f, 1.0f);
        Vec2 pivot = Body.GetPosition(RearWheelId);

        // float throttle = 0.0f;
        // float speed = 35.0f;
        // float torque = 2.5f * scale;
        // float hertz = 5.0f;
        // float dampingRatio = 0.7f;

        WheelJointDef jointDef = WheelJointDef.DefaultWheelJointDef();

        jointDef.BodyIdA = ChassisId;
        jointDef.BodyIdB = RearWheelId;
        jointDef.LocalAxisA = Body.GetLocalVector(jointDef.BodyIdA, axis);
        jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
        jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
        jointDef.MotorSpeed = 0.0f;
        jointDef.MaxMotorTorque = torque;
        jointDef.EnableMotor = true;
        jointDef.Hertz = hertz;
        jointDef.DampingRatio = dampingRatio;
        jointDef.LowerTranslation = -0.25f * scale;
        jointDef.UpperTranslation = 0.25f * scale;
        jointDef.EnableLimit = true;
        RearAxleId = Joint.CreateWheelJoint(worldId, jointDef);

        pivot = Body.GetPosition(FrontWheelId);
        jointDef.BodyIdA = ChassisId;
        jointDef.BodyIdB = FrontWheelId;
        jointDef.LocalAxisA = Body.GetLocalVector(jointDef.BodyIdA, axis);
        jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
        jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
        jointDef.MotorSpeed = 0.0f;
        jointDef.MaxMotorTorque = torque;
        jointDef.EnableMotor = true;
        jointDef.Hertz = hertz;
        jointDef.DampingRatio = dampingRatio;
        jointDef.LowerTranslation = -0.25f * scale;
        jointDef.UpperTranslation = 0.25f * scale;
        jointDef.EnableLimit = true;
        FrontAxleId = Joint.CreateWheelJoint(worldId, jointDef);
    }

    public void Despawn()
    {
        Debug.Assert(IsSpawned == true);

        Joint.DestroyJoint(RearAxleId);
        Joint.DestroyJoint(FrontAxleId);
        Body.DestroyBody(RearWheelId);
        Body.DestroyBody(FrontWheelId);
        Body.DestroyBody(ChassisId);

        IsSpawned = false;
    }

    public void SetSpeed(float speed)
    {
        WheelJointFunc.SetMotorSpeed(RearAxleId, speed);
        WheelJointFunc.SetMotorSpeed(FrontAxleId, speed);
        Joint.WakeBodies(RearAxleId);
    }

    public void SetTorque(float torque)
    {
        WheelJointFunc.SetMaxMotorTorque(RearAxleId, torque);
        WheelJointFunc.SetMaxMotorTorque(FrontAxleId, torque);
    }

    public void SetHertz(float hertz)
    {
        WheelJointFunc.SetSpringHertz(RearAxleId, hertz);
        WheelJointFunc.SetSpringHertz(FrontAxleId, hertz);
    }

    public void SetDampingRadio(float dampingRatio)
    {
        WheelJointFunc.SetSpringDampingRatio(RearAxleId, dampingRatio);
        WheelJointFunc.SetSpringDampingRatio(FrontAxleId, dampingRatio);
    }
}