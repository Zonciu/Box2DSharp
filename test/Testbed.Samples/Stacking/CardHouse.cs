using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Stacking;

[Sample("Stacking", "Card House")]
public class CardHouse : SampleBase
{
    public CardHouse(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.75f, 0.9f);
            Global.Camera.Zoom = 25.0f * 0.05f;
        }

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Position = (0.0f, -2.0f);
        BodyId groundId = Body.CreateBody(WorldId, bodyDef);

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Friction = 0.7f;

        Polygon groundBox = Geometry.MakeBox(40.0f, 2.0f);
        Shape.CreatePolygonShape(groundId, shapeDef, groundBox);

        float cardHeight = 0.2f;
        float cardThickness = 0.001f;

        float angle0 = 25.0f * B2Math.Pi / 180.0f;
        float angle1 = -25.0f * B2Math.Pi / 180.0f;
        float angle2 = 0.5f * B2Math.Pi;

        Polygon cardBox = Geometry.MakeBox(cardThickness, cardHeight);
        bodyDef.Type = BodyType.DynamicBody;

        int Nb = 5;
        float z0 = 0.0f;
        float y = cardHeight - 0.02f;
        while (Nb != 0)
        {
            float z = z0;
            for (int i = 0; i < Nb; i++)
            {
                if (i != Nb - 1)
                {
                    bodyDef.Position = (z + 0.25f, y + cardHeight - 0.015f);
                    bodyDef.Rotation = B2Math.MakeRot(angle2);
                    BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
                    Shape.CreatePolygonShape(bodyId, shapeDef, cardBox);
                }

                {
                    bodyDef.Position = (z, y);
                    bodyDef.Rotation = B2Math.MakeRot(angle1);
                    BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
                    Shape.CreatePolygonShape(bodyId, shapeDef, cardBox);

                    z += 0.175f;

                    bodyDef.Position = (z, y);
                    bodyDef.Rotation = B2Math.MakeRot(angle0);
                    bodyId = Body.CreateBody(WorldId, bodyDef);
                    Shape.CreatePolygonShape(bodyId, shapeDef, cardBox);

                    z += 0.175f;
                }
            }

            y += cardHeight * 2.0f - 0.03f;
            z0 += 0.175f;
            Nb--;
        }
    }
}