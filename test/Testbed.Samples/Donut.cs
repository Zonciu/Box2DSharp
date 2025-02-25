using System.Diagnostics;
using Box2DSharp;

namespace Testbed.Samples;

public class Donut
{
    public const int SideCount = 7;

    public BodyId[] BodyIds = new BodyId[SideCount];

    public JointId[] JointIds = new JointId[SideCount];

    public bool IsSpawned;

    public Donut()
    {
        for (int i = 0; i < SideCount; ++i)
        {
            BodyIds[i] = BodyId.NullId;
            JointIds[i] = JointId.NullId;
        }

        IsSpawned = false;
    }

    public void Spawn(WorldId worldId, Vec2 position, float scale, int groupIndex, object? userData)
    {
        Debug.Assert(IsSpawned == false);

        for (int i = 0; i < SideCount; ++i)
        {
            Debug.Assert(BodyIds[i].IsNull);
            Debug.Assert(JointIds[i].IsNull);
        }

        float radius = 1.0f * scale;
        float deltaAngle = 2.0f * B2Math.Pi / SideCount;
        float length = 2.0f * B2Math.Pi * radius / SideCount;

        Capsule capsule = ((0.0f, -0.5f * length), (0.0f, 0.5f * length), 0.25f * scale);

        Vec2 center = position;

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;
        bodyDef.UserData = userData;

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 1.0f;
        shapeDef.Filter.GroupIndex = -groupIndex;
        shapeDef.Friction = 0.3f;

        // Create bodies
        float angle = 0.0f;
        for (int i = 0; i < SideCount; ++i)
        {
            bodyDef.Position = (radius * MathF.Cos(angle) + center.X, radius * MathF.Sin(angle) + center.Y);
            bodyDef.Rotation = B2Math.MakeRot(angle);

            BodyIds[i] = Body.CreateBody(worldId, bodyDef);
            Shape.CreateCapsuleShape(BodyIds[i], shapeDef, capsule);

            angle += deltaAngle;
        }

        // Create joints
        WeldJointDef weldDef = WeldJointDef.DefaultWeldJointDef();
        weldDef.AngularHertz = 5.0f;
        weldDef.AngularDampingRatio = 0.0f;
        weldDef.LocalAnchorA = (0.0f, 0.5f * length);
        weldDef.LocalAnchorB = (0.0f, -0.5f * length);

        BodyId prevBodyId = BodyIds[SideCount - 1];
        for (int i = 0; i < SideCount; ++i)
        {
            weldDef.BodyIdA = prevBodyId;
            weldDef.BodyIdB = BodyIds[i];
            Rot rotA = Body.GetRotation(prevBodyId);
            Rot rotB = Body.GetRotation(BodyIds[i]);
            weldDef.ReferenceAngle = B2Math.RelativeAngle(rotB, rotA);
            JointIds[i] = Joint.CreateWeldJoint(worldId, weldDef);
            prevBodyId = weldDef.BodyIdB;
        }

        IsSpawned = true;
    }

    public void Despawn()
    {
        Debug.Assert(IsSpawned == true);

        for (int i = 0; i < SideCount; ++i)
        {
            Body.DestroyBody(BodyIds[i]);
            BodyIds[i] = BodyId.NullId;
            JointIds[i] = JointId.NullId;
        }

        IsSpawned = false;
    }
}