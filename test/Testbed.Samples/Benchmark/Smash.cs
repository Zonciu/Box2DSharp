using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Benchmark;

[Sample("Benchmark", "Smash")]
public class Smash : SampleBase
{
    public Smash(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (60.0f, 6.0f);
            Global.Camera.Zoom = 25.0f * 1.6f;
        }
    }

    public override void OnInitialized()
    {
        World.SetGravity(WorldId, Vec2.Zero);

        {
            Polygon box = Geometry.MakeBox(4.0f, 4.0f);

            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (-20.0f, 0.0f);
            bodyDef.LinearVelocity = (40.0f, 0.0f);
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Density = 8.0f;
            Shape.CreatePolygonShape(bodyId, shapeDef, box);
        }
        CreateScene();
        base.OnInitialized();
    }

    private void CreateScene()
    {
        float d = 0.4f;
        Polygon box = Geometry.MakeSquare(0.5f * d);

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;
        bodyDef.IsAwake = false;

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();

        int columns = Core.B2Debug ? 20 : 120;
        int rows = Core.B2Debug ? 10 : 80;

        for (int i = 0; i < columns; ++i)
        {
            for (int j = 0; j < rows; ++j)
            {
                bodyDef.Position.X = i * d + 30.0f;
                bodyDef.Position.Y = (j - rows / 2.0f) * d;
                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
                Shape.CreatePolygonShape(bodyId, shapeDef, box);
            }
        }
    }
}