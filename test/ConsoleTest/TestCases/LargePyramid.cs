using Box2DSharp;

namespace ConsoleTest.TestCases;

public class LargePyramid
{
    public WorldId WorldId;

    public float TimeStep = 1 / 60f;

    public void Init()
    {
        var worldDef = WorldDef.DefaultWorldDef();
        worldDef.WorkerCount = 1;

        //worldDef.EnqueueTask = EnqueueTaskCallback;
        //worldDef.FinishTask = FinishTaskCallback;
        worldDef.UserTaskContext = this;
        worldDef.EnableSleep = true;
        WorldId = World.CreateWorld(worldDef);
        int baseCount = 100;
        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Position = new(0.0f, -1.0f);
        BodyId groundId = Body.CreateBody(WorldId, bodyDef);

        Polygon box = Geometry.MakeBox(100.0f, 1.0f);
        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        Shape.CreatePolygonShape(groundId, shapeDef, box);

        bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;

        shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 1.0f;

        float h = 0.5f;
        box = Geometry.MakeRoundedBox(h - 0.05f, h - 0.05f, 0.05f);

        float shift = 1.0f * h;

        for (int i = 0; i < baseCount; ++i)
        {
            float y = (2.0f * i + 1.0f) * shift;

            for (int j = i; j < baseCount; ++j)
            {
                float x = (i + 1.0f) * shift + 2.0f * (j - i) * shift - h * baseCount;

                bodyDef.Position = new(x, y);

                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
                Shape.CreatePolygonShape(bodyId, shapeDef, box);
            }
        }
    }

    public void Step()
    {
        World.Step(WorldId, TimeStep, 4);
    }
}