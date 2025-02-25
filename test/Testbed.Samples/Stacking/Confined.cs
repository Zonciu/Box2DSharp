using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Stacking;

[Sample("Stacking", "Confined")]
public class Confined : SampleBase
{
    public const int e_gridCount = 25;

    public const int e_maxCount = e_gridCount * e_gridCount;

    public Confined(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 10.0f);
            Global.Camera.Zoom = 25.0f * 0.5f;
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Capsule capsule;
            capsule = ((-10.5f, 0.0f), (10.5f, 0.0f), 0.5f);
            Shape.CreateCapsuleShape(groundId, shapeDef, capsule);
            capsule = ((-10.5f, 0.0f), (-10.5f, 20.5f), 0.5f);
            Shape.CreateCapsuleShape(groundId, shapeDef, capsule);
            capsule = ((10.5f, 0.0f), (10.5f, 20.5f), 0.5f);
            Shape.CreateCapsuleShape(groundId, shapeDef, capsule);
            capsule = ((-10.5f, 20.5f), (10.5f, 20.5f), 0.5f);
            Shape.CreateCapsuleShape(groundId, shapeDef, capsule);
        }

        Row = 0;
        Column = 0;
        Count = 0;
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.GravityScale = 0.0f;

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Circle circle = ((0.0f, 0.0f), 0.5f);

            while (Count < e_maxCount)
            {
                Row = 0;
                for (int i = 0; i < e_gridCount; ++i)
                {
                    float x = -8.75f + Column * 18.0f / e_gridCount;
                    float y = 1.5f + Row * 18.0f / e_gridCount;

                    bodyDef.Position = (x, y);
                    BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
                    Shape.CreateCircleShape(bodyId, shapeDef, circle);

                    Count += 1;
                    Row += 1;
                }

                Column += 1;
            }
        }
    }

    int Row;

    int Column;

    int Count;
}