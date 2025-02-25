using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Robustness;

[Sample("Robustness", "HighMassRatio2")]
public class HighMassRatio2 : SampleBase
{
    public HighMassRatio2(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 16.5f);
            Global.Camera.Zoom = 25.0f;
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Polygon box = Geometry.MakeOffsetBox(50.0f, 1.0f, (0.0f, -1.0f), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();

            float extent = 1.0f;
            Vec2[] points = [(-0.5f * extent, 0.0f), (0.5f * extent, 0.0f), (0.0f, 1.0f * extent)];
            Hull hull = HullFunc.ComputeHull(points, 3);
            Polygon smallTriangle = Geometry.MakePolygon(hull, 0.0f);
            Polygon smallBox = Geometry.MakeBox(0.5f * extent, 0.5f * extent);
            Polygon bigBox = Geometry.MakeBox(10.0f * extent, 10.0f * extent);

            {
                bodyDef.Position = (-9.0f * extent, 0.5f * extent);
                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
                Shape.CreatePolygonShape(bodyId, shapeDef, smallBox);
            }

            {
                bodyDef.Position = (9.0f * extent, 0.5f * extent);
                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
                Shape.CreatePolygonShape(bodyId, shapeDef, smallBox);
            }

            {
                bodyDef.Position = (0.0f, (10.0f + 16.0f) * extent);
                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
                Shape.CreatePolygonShape(bodyId, shapeDef, bigBox);
            }
        }
    }
}