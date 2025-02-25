using System.Diagnostics;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Shapes;

[Sample("Shapes", "Modify Geometry")]
public class ModifyGeometry : SampleBase
{
    public ModifyGeometry(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Zoom = 25.0f * 0.25f;
            Global.Camera.Center = (0.0f, 5.0f);
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Polygon box = Geometry.MakeOffsetBox(10.0f, 1.0f, (0.0f, -1.0f), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (0.0f, 4.0f);
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Polygon box = Geometry.MakeBox(1.0f, 1.0f);
            Shape.CreatePolygonShape(bodyId, shapeDef, box);
        }

        {
            ShapeType = ShapeType.CircleShape;
            Scale = 1.0f;
            Circle = ((0.0f, 0.0f), 0.5f);
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.KinematicBody;
            bodyDef.Position = (0.0f, 1.0f);
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            ShapeId = Shape.CreateCircleShape(bodyId, shapeDef, Circle);
        }
    }

    protected void UpdateShape()
    {
        switch (ShapeType)
        {
        case ShapeType.CircleShape:
            Circle = ((0.0f, 0.0f), 0.5f * Scale);
            Shape.SetCircle(ShapeId, Circle);
            break;

        case ShapeType.CapsuleShape:
            Capsule = ((-0.5f * Scale, 0.0f), (0.0f, 0.5f * Scale), 0.5f * Scale);
            Shape.SetCapsule(ShapeId, Capsule);
            break;

        case ShapeType.SegmentShape:
            Segment = ((-0.5f * Scale, 0.0f), (0.75f * Scale, 0.0f));
            Shape.SetSegment(ShapeId, Segment);
            break;

        case ShapeType.PolygonShape:
            Polygon = Geometry.MakeBox(0.5f * Scale, 0.75f * Scale);
            Shape.SetPolygon(ShapeId, Polygon);
            break;

        default:
            Debug.Assert(false);
            break;
        }

        BodyId bodyId = Shape.GetBody(ShapeId);
        Body.ApplyMassFromShapes(bodyId);
    }

    protected ShapeId ShapeId;

    protected ShapeType ShapeType;

    protected float Scale;

    protected Circle Circle;

    protected Capsule Capsule;

    protected Segment Segment;

    protected Polygon Polygon;
}