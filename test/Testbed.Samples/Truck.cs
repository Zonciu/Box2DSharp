using System.Diagnostics;
using Box2DSharp;

namespace Testbed.Samples;

public class Truck
{
    public BodyId ChassisId;

    public BodyId RearWheelId;

    public BodyId FrontWheelId;

    public JointId RearAxleId;

    public JointId FrontAxleId;

    public bool IsSpawned;

    public Truck()
    {
        ChassisId = new();
        RearWheelId = new();
        FrontWheelId = new();
        RearAxleId = new();
        FrontAxleId = new();
        IsSpawned = false;
    }

    public void Spawn(
        WorldId worldId,
        Vec2 position,
        float scale,
        float hertz,
        float dampingRatio,
        float torque,
        float density,
        object? userData)
    {
        Debug.Assert(IsSpawned == false);

        Debug.Assert(ChassisId.IsNull);
        Debug.Assert(FrontWheelId.IsNull);
        Debug.Assert(RearWheelId.IsNull);

        // Vec2 vertices[6] = {
        //	( -1.5f, -0.5f ), ( 1.5f, -0.5f ), ( 1.5f, 0.0f ), ( 0.0f, 0.9f ), ( -1.15f, 0.9f ), ( -1.5f, 0.2f ),
        // };

        Vec2[] vertices = [(-0.65f, -0.4f), (1.5f, -0.4f), (1.5f, 0.0f), (0.0f, 0.9f), (-0.65f, 0.9f)];

        for (int i = 0; i < 5; ++i)
        {
            vertices[i].X *= 0.85f * scale;
            vertices[i].Y *= 0.85f * scale;
        }

        Hull hull = HullFunc.ComputeHull(vertices, 5);
        Polygon chassis = Geometry.MakePolygon(hull, 0.15f * scale);

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = density;
        shapeDef.Friction = 0.2f;
        shapeDef.CustomColor = (uint)B2HexColor.HotPink;

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;
        bodyDef.Position = (0.0f, 1.0f * scale) + position;
        ChassisId = Body.CreateBody(worldId, bodyDef);
        Shape.CreatePolygonShape(ChassisId, shapeDef, chassis);

        Polygon box = Geometry.MakeOffsetBox(1.25f * scale, 0.1f * scale, (-2.05f * scale, -0.275f * scale), Rot.Identity);
        box.Radius = 0.1f * scale;
        Shape.CreatePolygonShape(ChassisId, shapeDef, box);

        box = Geometry.MakeOffsetBox(0.05f * scale, 0.35f * scale, (-3.25f * scale, 0.375f * scale), Rot.Identity);
        box.Radius = 0.1f * scale;
        Shape.CreatePolygonShape(ChassisId, shapeDef, box);

        shapeDef.Density = 2.0f * density;
        shapeDef.Friction = 2.5f;
        shapeDef.CustomColor = (uint)B2HexColor.Silver;

        Circle circle = ((0.0f, 0.0f), 0.4f * scale);
        bodyDef.Position = (-2.75f * scale, 0.3f * scale) + position;
        RearWheelId = Body.CreateBody(worldId, bodyDef);
        Shape.CreateCircleShape(RearWheelId, shapeDef, circle);

        bodyDef.Position = (0.8f * scale, 0.3f * scale) + position;
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