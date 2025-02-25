using System.Diagnostics;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Benchmark;

[Sample("Benchmark", "ManyTumbler")]
public class ManyTumbler(Settings settings) : SampleBase(settings)
{
    protected BodyId GroundId;

    protected int RowCount;

    protected int ColumnCount;

    protected BodyId[] TumblerIds;

    protected Vec2[] Positions;

    protected int TumblerCount;

    protected BodyId[] BodyIds;

    protected int BodyCount;

    protected int BodyIndex;

    protected float AngularSpeed;

    public override void OnInitialized()
    {
        if (Settings.Restart == false)
        {
            Global.Camera.Center = (1, -5.5f);
            Global.Camera.Zoom = 25.0f * 3.4f;
            Settings.DrawJoints = false;
        }

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        GroundId = Body.CreateBody(WorldId, bodyDef);

        RowCount = Core.B2Debug ? 2 : 19;
        ColumnCount = Core.B2Debug ? 2 : 19;

        TumblerIds = null;
        Positions = null;
        TumblerCount = 0;

        BodyIds = null;
        BodyCount = 0;
        BodyIndex = 0;

        AngularSpeed = 25.0f;

        CreateScene();
    }

    void CreateTumbler(Vec2 position, int index)
    {
        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.KinematicBody;
        bodyDef.Position = (position.X, position.Y);
        bodyDef.AngularVelocity = B2Math.Pi / 180.0f * AngularSpeed;
        BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
        TumblerIds[index] = bodyId;

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 50.0f;

        Polygon polygon;
        polygon = Geometry.MakeOffsetBox(0.25f, 2.0f, (2.0f, 0.0f), Rot.Identity);
        Shape.CreatePolygonShape(bodyId, shapeDef, polygon);
        polygon = Geometry.MakeOffsetBox(0.25f, 2.0f, (-2.0f, 0.0f), Rot.Identity);
        Shape.CreatePolygonShape(bodyId, shapeDef, polygon);
        polygon = Geometry.MakeOffsetBox(2.0f, 0.25f, (0.0f, 2.0f), Rot.Identity);
        Shape.CreatePolygonShape(bodyId, shapeDef, polygon);
        polygon = Geometry.MakeOffsetBox(2.0f, 0.25f, (0.0f, -2.0f), Rot.Identity);
        Shape.CreatePolygonShape(bodyId, shapeDef, polygon);
    }

    protected void CreateScene()
    {
        for (int i = 0; i < BodyCount; ++i)
        {
            if (BodyIds[i].IsNotNull)
            {
                Body.DestroyBody(BodyIds[i]);
            }
        }

        for (int i = 0; i < TumblerCount; ++i)
        {
            Body.DestroyBody(TumblerIds[i]);
        }

        TumblerIds = null;
        Positions = null;

        TumblerCount = RowCount * ColumnCount;
        TumblerIds = new BodyId[TumblerCount];
        Positions = new Vec2[TumblerCount];

        int index = 0;
        float x = -4.0f * RowCount;
        for (int i = 0; i < RowCount; ++i)
        {
            float y = -4.0f * ColumnCount;
            for (int j = 0; j < ColumnCount; ++j)
            {
                Positions[index] = (x, y);
                CreateTumbler(Positions[index], index);
                ++index;
                y += 8.0f;
            }

            x += 8.0f;
        }

        BodyIds = null;

        int bodiesPerTumbler = Core.B2Debug ? 8 : 50;
        BodyCount = bodiesPerTumbler * TumblerCount;

        BodyIds = new BodyId[BodyCount];

        BodyIndex = 0;
    }

    public override void PostStep()
    {
        if (BodyIndex < BodyCount && (StepCount & 0x7) == 0)
        {
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();

            Capsule capsule = ((-0.1f, 0.0f), (0.1f, 0.0f), 0.075f);

            for (int i = 0; i < TumblerCount; ++i)
            {
                Debug.Assert(BodyIndex < BodyCount);

                BodyDef bodyDef = BodyDef.DefaultBodyDef();
                bodyDef.Type = BodyType.DynamicBody;
                bodyDef.Position = Positions[i];
                BodyIds[BodyIndex] = Body.CreateBody(WorldId, bodyDef);
                Shape.CreateCapsuleShape(BodyIds[BodyIndex], shapeDef, capsule);
                BodyIndex += 1;
            }
        }
    }
}