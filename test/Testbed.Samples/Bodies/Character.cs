using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Bodies;

/// This is a test of typical character collision scenarios. This does not
/// show how you should implement a character in your application.
/// Instead this is used to test smooth collision on chain shapes.
[Sample("Bodies", "Character")]
public class Character : SampleBase
{
    BodyId _circleCharacterId;

    BodyId _capsuleCharacterId;

    BodyId _boxCharacterId;

    public Character(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (-2.0f, 7.0f);
            Global.Camera.Zoom = 25.0f * 0.4f;
        }

        // Ground body
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Segment segment = ((-20.0f, 0.0f), (20.0f, 0.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }

        // Collinear edges with no adjacency information.
        // This shows the problematic case where a box shape can hit
        // an internal vertex.
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Segment segment1 = ((-8.0f, 1.0f), (-6.0f, 1.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment1);

            Segment segment2 = ((-6.0f, 1.0f), (-4.0f, 1.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment2);

            Segment segment3 = ((-4.0f, 1.0f), (-2.0f, 1.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment3);
        }

        // Chain shape
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Rotation = B2Math.MakeRot(0.25f * B2Math.Pi);
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            Vec2[] points = [(8.0f, 7.0f), (7.0f, 8.0f), (6.0f, 8.0f), (5.0f, 7.0f)];
            ChainDef chainDef = ChainDef.DefaultChainDef();
            chainDef.Points = points;
            chainDef.Count = 4;
            chainDef.IsLoop = true;

            Shape.CreateChain(groundId, chainDef);
        }

        // Square tiles. This shows that adjacency shapes may have non-smooth collision. Box2D has no solution
        // to this problem.
        // todo_erin try this: https://briansemrau.github.io/dealing-with-ghost-collisions/
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Polygon box = Geometry.MakeOffsetBox(1.0f, 1.0f, (4.0f, 3.0f), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);

            box = Geometry.MakeOffsetBox(1.0f, 1.0f, (6.0f, 3.0f), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);

            box = Geometry.MakeOffsetBox(1.0f, 1.0f, (8.0f, 3.0f), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);
        }

        // Square made from a chain loop. Collision should be smooth.
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            Vec2[] points = [(-1.0f, 3.0f), (1.0f, 3.0f), (1.0f, 5.0f), (-1.0f, 5.0f)];
            ChainDef chainDef = ChainDef.DefaultChainDef();
            chainDef.Points = points;
            chainDef.Count = 4;
            chainDef.IsLoop = true;
            Shape.CreateChain(groundId, chainDef);
        }

        // Chain loop. Collision should be smooth.
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Position = (-10.0f, 4.0f);
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            Vec2[] points =
            [
                (0.0f, 0.0f), (6.0f, 0.0f),
                (6.0f, 2.0f), (4.0f, 1.0f),
                (2.0f, 2.0f), (0.0f, 2.0f),
                (-2.0f, 2.0f), (-4.0f, 3.0f),
                (-6.0f, 2.0f), (-6.0f, 0.0f)
            ];
            ChainDef chainDef = ChainDef.DefaultChainDef();
            chainDef.Points = points;
            chainDef.Count = 10;
            chainDef.IsLoop = true;
            Shape.CreateChain(groundId, chainDef);
        }

        // Circle character
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Position = (-7.0f, 6.0f);
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.FixedRotation = true;
            bodyDef.EnableSleep = false;

            _circleCharacterId = Body.CreateBody(WorldId, bodyDef);

            Circle circle = ((0.0f, 0.0f), 0.25f);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Density = 20.0f;
            shapeDef.Friction = 0.2f;
            Shape.CreateCircleShape(_circleCharacterId, shapeDef, circle);
        }

        // Capsule character
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Position = (3.0f, 5.0f);
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.FixedRotation = true;
            bodyDef.EnableSleep = false;

            _capsuleCharacterId = Body.CreateBody(WorldId, bodyDef);

            Capsule capsule = ((0.0f, 0.25f), (0.0f, 0.75f), 0.25f);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Density = 20.0f;
            shapeDef.Friction = 0.2f;
            Shape.CreateCapsuleShape(_capsuleCharacterId, shapeDef, capsule);
        }

        // Square character
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Position = (-3.0f, 8.0f);
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.FixedRotation = true;
            bodyDef.EnableSleep = false;

            _boxCharacterId = Body.CreateBody(WorldId, bodyDef);

            Polygon box = Geometry.MakeBox(0.4f, 0.4f);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Density = 20.0f;
            shapeDef.Friction = 0.2f;
            Shape.CreatePolygonShape(_boxCharacterId, shapeDef, box);
        }
    }

    public override void UpdateUI()
    {
        DrawString("This tests various character collision shapes.");
        DrawString("Limitation: square and hexagon can snag on aligned boxes.");
        base.UpdateUI();
    }
}