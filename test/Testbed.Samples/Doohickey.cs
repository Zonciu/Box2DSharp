using System.Diagnostics;
using Box2DSharp;

namespace Testbed.Samples;

public class Doohickey
{
    public BodyId WheelId1;

    public BodyId WheelId2;

    public BodyId BarId1;

    public BodyId BarId2;

    public JointId AxleId1;

    public JointId AxleId2;

    public JointId SliderId;

    public bool IsSpawned;

    public Doohickey()
    {
        WheelId1 = new();
        WheelId2 = new();
        BarId1 = new();
        BarId2 = new();

        AxleId1 = new();
        AxleId2 = new();
        SliderId = new();

        IsSpawned = false;
    }

    public void Spawn(WorldId worldId, Vec2 position, float scale)
    {
        Debug.Assert(IsSpawned == false);

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        Circle circle = ((0.0f, 0.0f), 1.0f * scale);
        Capsule capsule = ((-3.5f * scale, 0.0f), (3.5f * scale, 0.0f), 0.15f * scale);

        bodyDef.Position = B2Math.MulAdd(position, scale, (-5.0f, 3.0f));
        WheelId1 = Body.CreateBody(worldId, bodyDef);
        Shape.CreateCircleShape(WheelId1, shapeDef, circle);

        bodyDef.Position = B2Math.MulAdd(position, scale, (5.0f, 3.0f));
        WheelId2 = Body.CreateBody(worldId, bodyDef);
        Shape.CreateCircleShape(WheelId2, shapeDef, circle);

        bodyDef.Position = B2Math.MulAdd(position, scale, (-1.5f, 3.0f));
        BarId1 = Body.CreateBody(worldId, bodyDef);
        Shape.CreateCapsuleShape(BarId1, shapeDef, capsule);

        bodyDef.Position = B2Math.MulAdd(position, scale, (1.5f, 3.0f));
        BarId2 = Body.CreateBody(worldId, bodyDef);
        Shape.CreateCapsuleShape(BarId2, shapeDef, capsule);

        RevoluteJointDef revoluteDef = RevoluteJointDef.DefaultRevoluteJointDef();

        revoluteDef.BodyIdA = WheelId1;
        revoluteDef.BodyIdB = BarId1;
        revoluteDef.LocalAnchorA = (0.0f, 0.0f);
        revoluteDef.LocalAnchorB = (-3.5f * scale, 0.0f);
        revoluteDef.EnableMotor = true;
        revoluteDef.MaxMotorTorque = 2.0f * scale;
        Joint.CreateRevoluteJoint(worldId, revoluteDef);

        revoluteDef.BodyIdA = WheelId2;
        revoluteDef.BodyIdB = BarId2;
        revoluteDef.LocalAnchorA = (0.0f, 0.0f);
        revoluteDef.LocalAnchorB = (3.5f * scale, 0.0f);
        revoluteDef.EnableMotor = true;
        revoluteDef.MaxMotorTorque = 2.0f * scale;
        Joint.CreateRevoluteJoint(worldId, revoluteDef);

        PrismaticJointDef prismaticDef = PrismaticJointDef.DefaultPrismaticJointDef();
        prismaticDef.BodyIdA = BarId1;
        prismaticDef.BodyIdB = BarId2;
        prismaticDef.LocalAxisA = (1.0f, 0.0f);
        prismaticDef.LocalAnchorA = (2.0f * scale, 0.0f);
        prismaticDef.LocalAnchorB = (-2.0f * scale, 0.0f);
        prismaticDef.LowerTranslation = -2.0f * scale;
        prismaticDef.UpperTranslation = 2.0f * scale;
        prismaticDef.EnableLimit = true;
        prismaticDef.EnableMotor = true;
        prismaticDef.MaxMotorForce = 2.0f * scale;
        prismaticDef.EnableSpring = true;
        prismaticDef.Hertz = 1.0f;
        prismaticDef.DampingRatio = 0.5f;
        Joint.CreatePrismaticJoint(worldId, prismaticDef);
    }

    public void Despawn()
    {
        Debug.Assert(IsSpawned);

        Joint.DestroyJoint(AxleId1);
        Joint.DestroyJoint(AxleId2);
        Joint.DestroyJoint(SliderId);

        Body.DestroyBody(WheelId1);
        Body.DestroyBody(WheelId2);
        Body.DestroyBody(BarId1);
        Body.DestroyBody(BarId2);

        IsSpawned = false;
    }
}