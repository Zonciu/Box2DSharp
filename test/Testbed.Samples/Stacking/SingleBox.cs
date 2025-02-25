using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Stacking;

[Sample("Stacking", "SingleBox")]
public class SingleBox : SampleBase
{
    public SingleBox(Settings settings)
        : base(settings)
    {
        float extent = 1.0f;

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        BodyId groundId = Body.CreateBody(WorldId, bodyDef);

        float groundWidth = 66.0f * extent;
        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Friction = 0.5f;

        Segment segment = new(new(-0.5f * 2.0f * groundWidth, 0.0f), new(0.5f * 2.0f * groundWidth, 0.0f));
        Shape.CreateSegmentShape(groundId, shapeDef, segment);
        bodyDef.Type = BodyType.DynamicBody;
        {
            Polygon box = Geometry.MakeBox(extent, extent);
            bodyDef.Position = new(0.0f, 4.0f);
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
            Shape.CreatePolygonShape(bodyId, shapeDef, box);
        }
    }
}