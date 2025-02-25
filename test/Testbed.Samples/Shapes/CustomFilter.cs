using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Shapes;

[Sample("Shapes", "Custom Filter")]
public class CustomFilter : SampleBase
{
    public const int e_count = 10;

    public CustomFilter(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 5.0f);
            Global.Camera.Zoom = 10.0f;
        }

        // Register custom filter
        World.SetCustomFilterCallback(WorldId, CustomFilterStatic, this);

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);
            Segment segment = ((-40.0f, 0.0f), (40.0f, 0.0f));

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();

            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Polygon box = Geometry.MakeSquare(1.0f);
            float x = -e_count;

            for (int i = 0; i < e_count; ++i)
            {
                bodyDef.Position = (x, 5.0f);
                BodyIds[i] = Body.CreateBody(WorldId, bodyDef);

                shapeDef.UserData = i + 1;
                ShapeIds[i] = Shape.CreatePolygonShape(BodyIds[i], shapeDef, box);
                x += 2.0f;
            }
        }
    }

    protected override void OnRender()
    {
        DrawString("Custom filter disables collision between odd and even shapes");

        for (int i = 0; i < e_count; ++i)
        {
            Vec2 p = Body.GetPosition(BodyIds[i]);
            Draw.DrawString(p, i.ToString());
        }
    }

    bool ShouldCollide(ShapeId shapeIdA, ShapeId shapeIdB)
    {
        object? userDataA = Shape.GetUserData(shapeIdA);
        object? userDataB = Shape.GetUserData(shapeIdB);

        if (userDataA == null || userDataB == null)
        {
            return true;
        }

        int indexA = (int)(userDataA);
        int indexB = (int)(userDataB);

        return (indexA & 1) + (indexB & 1) != 1;
    }

    static bool CustomFilterStatic(ShapeId shapeIdA, ShapeId shapeIdB, object context)
    {
        CustomFilter customFilter = (CustomFilter)(context);
        return customFilter.ShouldCollide(shapeIdA, shapeIdB);
    }

    BodyId[] BodyIds = new BodyId[e_count];

    ShapeId[] ShapeIds = new ShapeId[e_count];
}