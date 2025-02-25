using System;
using Box2DSharp;
using ConsoleTest.Base;

namespace ConsoleTest.TestCases;

public class LargeWorld : IDisposable
{
    protected Vec2 ViewPosition;

    protected float Period;

    protected int CycleCount;

    protected int CycleIndex;

    protected float GridCount;

    protected float GridSize;

    protected float Speed;

    protected Vec2 ExplosionPosition;

    protected bool Explode;

    protected bool FollowCar;

    WorldId WorldId;

    public LargeWorld()
    {
        Period = 40.0f;
        float omega = 2.0f * B2Math.Pi / Period;
        CycleCount = Core.B2Debug ? 10 : 100;
        GridSize = 1.0f;
        GridCount = (int)(CycleCount * Period / GridSize);

        float xStart = -0.5f * (CycleCount * Period);

        ViewPosition = (xStart, 15.0f);
        var worldDef = WorldDef.DefaultWorldDef();
        worldDef.WorkerCount = 8;
        worldDef.UserTaskContext = this;
        worldDef.EnableSleep = true;
        WorldId = World.CreateWorld(worldDef);

        // if (settings.Restart == false)
        // {
        //     Global.Camera.Center = ViewPosition;
        //     Global.Camera.Zoom = 25.0f * 1.0f;
        //     settings.DrawJoints = false;
        //     settings.UseCameraBounds = true;
        // }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();

            // Setting this to false significantly reduces the cost of creating
            // static bodies and shapes.
            shapeDef.ForceContactCreation = false;

            float height = 4.0f;
            float xBody = xStart;
            float xShape = xStart;

            BodyId groundId = new();

            for (int i = 0; i < GridCount; ++i)
            {
                // Create a new body regularly so that shapes are not too far from the body origin.
                // Most algorithms in Box2D work in local coordinates, but contact points are computed
                // relative to the body origin.
                // This makes a noticeable improvement in stability far from the origin.
                if (i % 10 == 0)
                {
                    bodyDef.Position.X = xBody;
                    groundId = Body.CreateBody(WorldId, bodyDef);
                    xShape = 0.0f;
                }

                float y = 0.0f;

                int ycount = (int)MathF.Round(height * MathF.Cos(omega * xBody)) + 12;

                for (int j = 0; j < ycount; ++j)
                {
                    Polygon square = Geometry.MakeOffsetBox(0.4f * GridSize, 0.4f * GridSize, (xShape, y), Rot.Identity);
                    square.Radius = 0.1f;
                    Shape.CreatePolygonShape(groundId, shapeDef, square);

                    y += GridSize;
                }

                xBody += GridSize;
                xShape += GridSize;
            }
        }

        int humanIndex = 0;
        int donutIndex = 0;
        for (int cycleIndex = 0; cycleIndex < CycleCount; ++cycleIndex)
        {
            float xbase = (0.5f + cycleIndex) * Period + xStart;

            int remainder = cycleIndex % 3;
            if (remainder == 0)
            {
                BodyDef bodyDef = BodyDef.DefaultBodyDef();
                bodyDef.Type = BodyType.DynamicBody;
                bodyDef.Position = (xbase - 3.0f, 10.0f);

                ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
                Polygon box = Geometry.MakeBox(0.3f, 0.2f);

                for (int i = 0; i < 10; ++i)
                {
                    bodyDef.Position.Y = 10.0f;
                    for (int j = 0; j < 5; ++j)
                    {
                        BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
                        Shape.CreatePolygonShape(bodyId, shapeDef, box);
                        bodyDef.Position.Y += 0.5f;
                    }

                    bodyDef.Position.X += 0.6f;
                }
            }
            else if (remainder == 1)
            {
                Vec2 position = (xbase - 2.0f, 10.0f);
                for (int i = 0; i < 5; ++i)
                {
                    Human human = new();
                    human.Spawn(WorldId, position, 1.5f, 0.05f, 0.0f, 0.0f, humanIndex + 1, null, false);
                    humanIndex += 1;
                    position.X += 1.0f;
                }
            }
            else
            {
                Vec2 position = (xbase - 4.0f, 12.0f);

                for (int i = 0; i < 5; ++i)
                {
                    Donut donut = new();
                    donut.Spawn(WorldId, position, 0.75f, 0, null);
                    donutIndex += 1;
                    position.X += 2.0f;
                }
            }
        }

        Car Car = new();
        Car.Spawn(WorldId, (xStart + 20.0f, 40.0f), 10.0f, 2.0f, 0.7f, 2000.0f, null);

        CycleIndex = 0;
        Speed = 0.0f;
        ExplosionPosition = ((0.5f + CycleIndex) * Period + xStart, 7.0f);
        Explode = true;
        FollowCar = false;
    }

    public void Step()
    {
        World.Step(WorldId, 1 / 60f, 4);
    }

    public void Dispose()
    {
        World.DestroyWorld(WorldId);
        WorldId = WorldId.NullId;
    }
}