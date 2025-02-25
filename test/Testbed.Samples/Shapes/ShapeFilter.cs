using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Shapes;

[Sample("Shapes", "Filter")]
public class ShapeFilter : SampleBase
{
    protected static class CollisionBits
    {
        public const uint Ground = 0x00000001;

        public const uint Team1 = 0x00000002;

        public const uint Team2 = 0x00000004;

        public const uint Team3 = 0x00000008;

        public const uint AllBits = ~0u;
    }

    public ShapeFilter(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Zoom = 25.0f * 0.5f;
            Global.Camera.Center = (0.0f, 5.0f);
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);
            Segment segment = ((-20.0f, 0.0f), (20.0f, 0.0f));

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Filter.CategoryBits = CollisionBits.Ground;
            shapeDef.Filter.MaskBits = CollisionBits.AllBits;

            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;

            bodyDef.Position = (0.0f, 2.0f);
            Player1Id = Body.CreateBody(WorldId, bodyDef);

            bodyDef.Position = (0.0f, 5.0f);
            Player2Id = Body.CreateBody(WorldId, bodyDef);

            bodyDef.Position = (0.0f, 8.0f);
            Player3Id = Body.CreateBody(WorldId, bodyDef);

            Polygon box = Geometry.MakeBox(2.0f, 1.0f);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();

            shapeDef.Filter.CategoryBits = CollisionBits.Team1;
            shapeDef.Filter.MaskBits = CollisionBits.Ground | CollisionBits.Team2 | CollisionBits.Team3;
            Shape1Id = Shape.CreatePolygonShape(Player1Id, shapeDef, box);

            shapeDef.Filter.CategoryBits = CollisionBits.Team2;
            shapeDef.Filter.MaskBits = CollisionBits.Ground | CollisionBits.Team1 | CollisionBits.Team3;
            Shape2Id = Shape.CreatePolygonShape(Player2Id, shapeDef, box);

            shapeDef.Filter.CategoryBits = CollisionBits.Team3;
            shapeDef.Filter.MaskBits = CollisionBits.Ground | CollisionBits.Team1 | CollisionBits.Team2;
            Shape3Id = Shape.CreatePolygonShape(Player3Id, shapeDef, box);
        }
    }

    protected override void OnRender()
    {
        Vec2 p1 = Body.GetPosition(Player1Id);
        Draw.DrawString((p1.X - 0.5f, p1.Y), "player 1");

        Vec2 p2 = Body.GetPosition(Player2Id);
        Draw.DrawString((p2.X - 0.5f, p2.Y), "player 2");

        Vec2 p3 = Body.GetPosition(Player3Id);
        Draw.DrawString((p3.X - 0.5f, p3.Y), "player 3");
    }

    protected BodyId Player1Id;

    protected BodyId Player2Id;

    protected BodyId Player3Id;

    protected ShapeId Shape1Id;

    protected ShapeId Shape2Id;

    protected ShapeId Shape3Id;
}