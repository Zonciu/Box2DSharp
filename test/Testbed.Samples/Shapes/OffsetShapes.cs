using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Shapes;

[Sample("Shapes", "Offset Shapes")]
public class OffsetShapes : SampleBase
{
    public OffsetShapes(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Zoom = 25.0f * 0.55f;
            Global.Camera.Center = (2.0f, 8.0f);
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Position = (-1.0f, 1.0f);
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Polygon box = Geometry.MakeOffsetBox(1.0f, 1.0f, (10.0f, -2.0f), B2Math.MakeRot(0.5f * B2Math.Pi));
            Shape.CreatePolygonShape(groundId, shapeDef, box);
        }

        {
            Capsule capsule = ((-5.0f, 1.0f), (-4.0f, 1.0f), 0.25f);
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Position = (13.5f, -0.75f);
            bodyDef.Type = BodyType.DynamicBody;
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreateCapsuleShape(bodyId, shapeDef, capsule);
        }

        {
            Polygon box = Geometry.MakeOffsetBox(0.75f, 0.5f, (9.0f, 2.0f), B2Math.MakeRot(0.5f * B2Math.Pi));
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Position = (0.0f, 0.0f);
            bodyDef.Type = BodyType.DynamicBody;
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreatePolygonShape(bodyId, shapeDef, box);
        }
    }

    protected override void OnRender()
    {
        Draw.DrawTransform(Transform.Identity);
    }
}