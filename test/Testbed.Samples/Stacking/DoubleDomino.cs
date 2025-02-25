using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Stacking;

[Sample("Stacking", "Double Domino")]
public class DoubleDomino : SampleBase
{
    public DoubleDomino(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 4.0f);
            Global.Camera.Zoom = 25.0f * 0.25f;
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Position = (0.0f, -1.0f);
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            Polygon box = Geometry.MakeBox(100.0f, 1.0f);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreatePolygonShape(groundId, shapeDef, box);
        }
        {
            Polygon box = Geometry.MakeBox(0.125f, 0.5f);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Friction = 0.6f;
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;

            int count = 15;
            float x = -0.5f * count;
            for (int i = 0; i < count; ++i)
            {
                bodyDef.Position = (x, 0.5f);
                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
                Shape.CreatePolygonShape(bodyId, shapeDef, box);
                if (i == 0)
                {
                    Body.ApplyLinearImpulse(bodyId, (0.2f, 0.0f), (x, 1.0f), true);
                }

                x += 1.0f;
            }
        }
    }
}