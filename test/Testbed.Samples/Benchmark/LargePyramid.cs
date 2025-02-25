using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Benchmark;

[Sample("Benchmark", "LargePyramid")]
public class LargePyramid(Settings settings) : SampleBase(settings)
{
    public override void OnInitialized()
    {
        if (Settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 50.0f);
            Global.Camera.Zoom = 25.0f * 2.2f;
        }

        int baseCount = Core.B2Debug ? 3 : 100;
        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Position = new(0.0f, -1.0f);
        BodyId groundId = Body.CreateBody(WorldId, bodyDef);

        Polygon box = Geometry.MakeBox(100.0f, 1.0f);
        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        Shape.CreatePolygonShape(groundId, shapeDef, box);

        bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;

        shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 1.0f;

        float h = 0.5f;
        box = Geometry.MakeRoundedBox(h - 0.05f, h - 0.05f, 0.05f);

        float shift = 1.0f * h;

        for (int i = 0; i < baseCount; ++i)
        {
            float y = (2.0f * i + 1.0f) * shift;

            for (int j = i; j < baseCount; ++j)
            {
                float x = (i + 1.0f) * shift + 2.0f * (j - i) * shift - h * baseCount;

                bodyDef.Position = new(x, y);

                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
                Shape.CreatePolygonShape(bodyId, shapeDef, box);
            }
        }
    }
}