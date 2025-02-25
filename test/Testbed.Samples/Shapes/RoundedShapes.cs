using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Shapes;

[Sample("Shapes", "Rounded")]
public class RoundedShapes : SampleBase
{
    public RoundedShapes(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Zoom = 25.0f * 0.55f;
            Global.Camera.Center = (2.0f, 8.0f);
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Polygon box = Geometry.MakeOffsetBox(20.0f, 1.0f, (0.0f, -1.0f), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);

            box = Geometry.MakeOffsetBox(1.0f, 5.0f, (19.0f, 5.0f), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);

            box = Geometry.MakeOffsetBox(1.0f, 5.0f, (-19.0f, 5.0f), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);
        }

        // Capsule capsule = ((-0.25f, 0.0f), (0.25f, 0.0f), 0.25f);
        // Circle circle = ((0.0f, 0.0f), 0.35f);
        // Polygon square = Geometry.MakeSquare(0.35f);

        // Vec2 points[3] = ((-0.1f, -0.5f), (0.1f, -0.5f), (0.0f, 0.5f));
        // Hull wedgeHull = HullFunc.ComputeHull(points, 3);
        // Polygon wedge = Geometry.MakePolygon(wedgeHull, 0.0f);
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();

            float y = 2.0f;
            int xcount = 10, ycount = 10;

            for (int i = 0; i < ycount; ++i)
            {
                float x = -5.0f;
                for (int j = 0; j < xcount; ++j)
                {
                    bodyDef.Position = (x, y);
                    BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

                    Polygon poly = B2Random.Shared.RandomPolygon(0.5f);
                    poly.Radius = B2Random.Shared.RandomFloat(0.05f, 0.25f);
                    Shape.CreatePolygonShape(bodyId, shapeDef, poly);

                    x += 1.0f;
                }

                y += 1.0f;
            }
        }
    }
}