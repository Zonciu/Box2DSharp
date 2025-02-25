using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Benchmark;

[Sample("Benchmark", "Compound")]
public class Compound : SampleBase
{
    public Compound(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (18.0f, 115.0f);
            Global.Camera.Zoom = 25.0f * 5.5f;
        }

        float grid = 1.0f;

        int height = Core.B2Debug ? 100 : 200;
        int width = Core.B2Debug ? 100 : 200;

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();

            for (int i = 0; i < height; ++i)
            {
                float y = grid * i;
                for (int j = i; j < width; ++j)
                {
                    float x = grid * j;
                    Polygon square = Geometry.MakeOffsetBox(0.5f * grid, 0.5f * grid, (x, y), Rot.Identity);
                    Shape.CreatePolygonShape(groundId, shapeDef, square);
                }
            }

            for (int i = 0; i < height; ++i)
            {
                float y = grid * i;
                for (int j = i; j < width; ++j)
                {
                    float x = -grid * j;
                    Polygon square = Geometry.MakeOffsetBox(0.5f * grid, 0.5f * grid, (x, y), Rot.Identity);
                    Shape.CreatePolygonShape(groundId, shapeDef, square);
                }
            }
        }

        {
            int span = Core.B2Debug ? 5 : 20;
            int count = 5;

            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;

            // defer mass properties to avoid n-squared mass computations
            bodyDef.AutomaticMass = false;
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();

            for (int m = 0; m < count; ++m)
            {
                float ybody = (100.0f + m * span) * grid;

                for (int n = 0; n < count; ++n)
                {
                    float xbody = -0.5f * grid * count * span + n * span * grid;
                    bodyDef.Position = (xbody, ybody);
                    BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

                    for (int i = 0; i < span; ++i)
                    {
                        float y = i * grid;
                        for (int j = 0; j < span; ++j)
                        {
                            float x = j * grid;
                            Polygon square = Geometry.MakeOffsetBox(0.5f * grid, 0.5f * grid, (x, y), Rot.Identity);
                            Shape.CreatePolygonShape(bodyId, shapeDef, square);
                        }
                    }

                    // All shapes have been added so I can efficiently compute the mass properties.
                    Body.ApplyMassFromShapes(bodyId);
                }
            }
        }
    }
}