using System.Diagnostics;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Robustness;

[Sample("Robustness", "Overlap Recovery")]
public class OverlapRecovery : SampleBase
{
    public OverlapRecovery(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 2.5f);
            Global.Camera.Zoom = 25.0f * 0.15f;
        }

        BodyIds = null;
        BodyCount = 0;
        BaseCount = 4;
        Overlap = 0.25f;
        Extent = 0.5f;
        PushOut = 3.0f;
        Hertz = 30.0f;
        DampingRatio = 10.0f;

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        BodyId groundId = Body.CreateBody(WorldId, bodyDef);

        float groundWidth = 40.0f;
        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 1.0f;

        Segment segment = ((-groundWidth, 0.0f), (groundWidth, 0.0f));
        Shape.CreateSegmentShape(groundId, shapeDef, segment);

        CreateScene();
    }

    protected void CreateScene()
    {
        for (int i = 0; i < BodyCount; ++i)
        {
            Body.DestroyBody(BodyIds[i]);
        }

        World.SetContactTuning(WorldId, Hertz, DampingRatio, PushOut);

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;

        Polygon box = Geometry.MakeBox(Extent, Extent);
        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 1.0f;

        BodyCount = BaseCount * (BaseCount + 1) / 2;
        BodyIds = new BodyId[BodyCount];

        int bodyIndex = 0;
        float fraction = 1.0f - Overlap;
        float y = Extent;
        for (int i = 0; i < BaseCount; ++i)
        {
            float x = fraction * Extent * (i - BaseCount);
            for (int j = i; j < BaseCount; ++j)
            {
                bodyDef.Position = (x, y);
                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

                Shape.CreatePolygonShape(bodyId, shapeDef, box);

                BodyIds[bodyIndex++] = bodyId;

                x += 2.0f * fraction * Extent;
            }

            y += 2.0f * fraction * Extent;
        }

        Debug.Assert(bodyIndex == BodyCount);
    }

    protected BodyId[] BodyIds;

    protected int BodyCount;

    protected int BaseCount;

    protected float Overlap;

    protected float Extent;

    protected float PushOut;

    protected float Hertz;

    protected float DampingRatio;
}