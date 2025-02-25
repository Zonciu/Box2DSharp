using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Stacking;

[Sample("Stacking", "Tilted Stack")]
public class TiltedStack : SampleBase
{
    public const int e_columns = 10;

    public const int e_rows = 10;

    public TiltedStack(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (7.5f, 7.5f);
            Global.Camera.Zoom = 20.0f;
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Position = (0.0f, -1.0f);
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            Polygon box = Geometry.MakeBox(1000.0f, 1.0f);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreatePolygonShape(groundId, shapeDef, box);
        }

        for (int i = 0; i < e_rows * e_columns; ++i)
        {
            Bodies[i] = BodyId.NullId;
        }

        {
            Polygon box = Geometry.MakeRoundedBox(0.45f, 0.45f, 0.05f);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Density = 1.0f;
            shapeDef.Friction = 0.3f;

            float offset = 0.2f;
            float dx = 5.0f;
            float xroot = -0.5f * dx * (e_columns - 1.0f);

            for (int j = 0; j < e_columns; ++j)
            {
                float x = xroot + j * dx;

                for (int i = 0; i < e_rows; ++i)
                {
                    BodyDef bodyDef = BodyDef.DefaultBodyDef();
                    bodyDef.Type = BodyType.DynamicBody;

                    int n = j * e_rows + i;

                    bodyDef.Position = (x + offset * i, 0.5f + 1.0f * i);
                    BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

                    Bodies[n] = bodyId;

                    Shape.CreatePolygonShape(bodyId, shapeDef, box);
                }
            }
        }
    }

    BodyId[] Bodies = new BodyId[e_rows * e_columns];
}