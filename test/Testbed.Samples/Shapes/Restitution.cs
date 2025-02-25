using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Shapes;

[Sample("Shapes", "Restitution")]
public class Restitution : SampleBase
{
    public const int e_count = 40;

    public const int e_circleShape = 0;

    public const int e_boxShape = 1;

    public Restitution(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (4.0f, 17.0f);
            Global.Camera.Zoom = 27.5f;
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            float h = 1.0f * e_count;
            Segment segment = ((-h, 0.0f), (h, 0.0f));
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }

        for (int i = 0; i < e_count; ++i)
        {
            BodyIds[i] = BodyId.NullId;
        }

        ShapeType = e_circleShape;

        CreateBodies();
    }

    protected void CreateBodies()
    {
        for (int i = 0; i < e_count; ++i)
        {
            if (BodyIds[i].IsNotNull)
            {
                Body.DestroyBody(BodyIds[i]);
                BodyIds[i] = BodyId.NullId;
            }
        }

        Circle circle = new();
        circle.Radius = 0.5f;

        Polygon box = Geometry.MakeBox(0.5f, 0.5f);

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 1.0f;
        shapeDef.Restitution = 0.0f;

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;

        float dr = 1.0f / (e_count > 1 ? e_count - 1 : 1);
        float x = -1.0f * (e_count - 1);
        float dx = 2.0f;

        for (int i = 0; i < e_count; ++i)
        {
            bodyDef.Position = (x, 40.0f);
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            BodyIds[i] = bodyId;

            if (ShapeType == e_circleShape)
            {
                Shape.CreateCircleShape(bodyId, shapeDef, circle);
            }
            else
            {
                Shape.CreatePolygonShape(bodyId, shapeDef, box);
            }

            shapeDef.Restitution += dr;
            x += dx;
        }
    }

    protected BodyId[] BodyIds = new BodyId[e_count];

    protected int ShapeType;
}