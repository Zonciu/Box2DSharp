using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Shapes;

[Sample("Shapes", "Chain Link")]
public class ChainLink : SampleBase
{
    public ChainLink(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 5.0f);
            Global.Camera.Zoom = 25.0f * 0.5f;
        }

        Vec2[] points1 =
        [
            (40.0f, 1.0f), (0.0f, 0.0f), (-40.0f, 0.0f),
            (-40.0f, -1.0f), (0.0f, -1.0f), (40.0f, -1.0f)
        ];
        Vec2[] points2 =
        [
            (-40.0f, -1.0f), (0.0f, -1.0f), (40.0f, -1.0f),
            (40.0f, 0.0f), (0.0f, 0.0f), (-40.0f, 0.0f)
        ];

        int count1 = points1.Length;
        int count2 = points2.Length;

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        BodyId groundId = Body.CreateBody(WorldId, bodyDef);

        {
            ChainDef chainDef = ChainDef.DefaultChainDef();
            chainDef.Points = points1;
            chainDef.Count = count1;
            chainDef.IsLoop = false;
            Shape.CreateChain(groundId, chainDef);
        }

        {
            ChainDef chainDef = ChainDef.DefaultChainDef();
            chainDef.Points = points2;
            chainDef.Count = count2;
            chainDef.IsLoop = false;
            Shape.CreateChain(groundId, chainDef);
        }

        bodyDef.Type = BodyType.DynamicBody;
        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();

        {
            bodyDef.Position = (-5.0f, 2.0f);
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
            Circle circle = ((0.0f, 0.0f), 0.5f);
            Shape.CreateCircleShape(bodyId, shapeDef, circle);
        }

        {
            bodyDef.Position = (0.0f, 2.0f);
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
            Capsule capsule = ((-0.5f, 0.0f), (0.5f, 0.0f), 0.25f);
            Shape.CreateCapsuleShape(bodyId, shapeDef, capsule);
        }

        {
            bodyDef.Position = (5.0f, 2.0f);
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
            float h = 0.5f;
            Polygon box = Geometry.MakeBox(h, h);
            Shape.CreatePolygonShape(bodyId, shapeDef, box);
        }
    }

    protected override void OnRender()
    {
        DrawString("This shows how to link together two chain shapes");
    }
}