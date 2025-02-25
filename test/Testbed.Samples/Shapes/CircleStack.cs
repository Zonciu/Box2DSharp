using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Shapes;

[Sample("Stacking", "Circle Stack")]
public class CircleStack : SampleBase
{
    public CircleStack(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 2.0f);
            Global.Camera.Zoom = 3.0f;
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Segment segment = ((-10.0f, 0.0f), (10.0f, 0.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }

        World.SetGravity(WorldId, (0.0f, -20.0f));
        World.SetContactTuning(WorldId, 0.25f * 360.0f, 10.0f, 3.0f);
        {
            Circle circle = new();
            circle.Radius = 0.5f;

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;

            float y = 0.5f;

            for (int i = 0; i < 8; ++i)
            {
                bodyDef.Position.Y = y;

                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
                Shape.CreateCircleShape(bodyId, shapeDef, circle);

                y += 1.0f;
            }
        }
    }
}