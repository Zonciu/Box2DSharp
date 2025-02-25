using System.Diagnostics;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Benchmark;

[Sample("Benchmark", "ManyPyramids")]
public class ManyPyramids : SampleBase
{
    protected BodyId GroundId;

    protected BodyId[] BodyIds;

    protected int BodyCount;

    protected int BodyIndex;

    protected int BaseCount;

    protected int RowCount;

    protected int ColumnCount;

    protected float Round;

    protected float Extent;

    public ManyPyramids(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (16.0f, 110.0f);
            Global.Camera.Zoom = 25.0f * 5.0f;
        }

        Extent = 0.5f;
        Round = 0.0f;
        BaseCount = 10;
        RowCount = Core.B2Debug ? 4 : 13;
        ColumnCount = Core.B2Debug ? 4 : 14;
        GroundId = BodyId.NullId;
        BodyIds = null!;
        BodyCount = 0;
        BodyIndex = 0;

        CreateScene();
    }

    public void CreatePyramid(float centerX, float baseY)
    {
        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 1.0f;

        float h = Extent - Round;
        Polygon box = Geometry.MakeRoundedBox(h, h, Round);

        float shift = 1.0f * h;

        for (int i = 0; i < BaseCount; ++i)
        {
            float y = (2.0f * i + 1.0f) * shift + baseY;

            for (int j = i; j < BaseCount; ++j)
            {
                float x = (i + 1.0f) * shift + 2.0f * (j - i) * shift + centerX - 0.5f;

                bodyDef.Position = (x, y);

                Debug.Assert(BodyIndex < BodyCount);
                BodyIds[BodyIndex] = Body.CreateBody(WorldId, bodyDef);
                Shape.CreatePolygonShape(BodyIds[BodyIndex], shapeDef, box);

                BodyIndex += 1;
            }
        }
    }

    public void CreateScene()
    {
        if (GroundId.IsNotNull)
        {
            Body.DestroyBody(GroundId);
        }

        for (int i = 0; i < BodyCount; ++i)
        {
            Body.DestroyBody(BodyIds[i]);
        }

        BodyIds = null;

        BodyCount = RowCount * ColumnCount * BaseCount * (BaseCount + 1) / 2;
        BodyIds = new BodyId[BodyCount];
        BodyIndex = 0;

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        GroundId = Body.CreateBody(WorldId, bodyDef);

        float groundDeltaY = 2.0f * Extent * (BaseCount + 1.0f);
        float groundWidth = 2.0f * Extent * ColumnCount * (BaseCount + 1.0f);
        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();

        float groundY = 0.0f;

        for (int i = 0; i < RowCount; ++i)
        {
            // Segment segment = {{-0.5f * groundWidth, groundY}, {0.5f * groundWidth, groundY}};
            Segment segment = ((-0.5f * 2.0f * groundWidth, groundY), (0.5f * 2.0f * groundWidth, groundY));
            Shape.CreateSegmentShape(GroundId, shapeDef, segment);
            groundY += groundDeltaY;
        }

        float baseWidth = 2.0f * Extent * BaseCount;
        float baseY = 0.0f;

        for (int i = 0; i < RowCount; ++i)
        {
            for (int j = 0; j < ColumnCount; ++j)
            {
                float centerX = -0.5f * groundWidth + j * (baseWidth + 2.0f * Extent) + Extent;
                CreatePyramid(centerX, baseY);
            }

            baseY += groundDeltaY;
        }

        Debug.Assert(BodyIndex == BodyCount);
    }
}