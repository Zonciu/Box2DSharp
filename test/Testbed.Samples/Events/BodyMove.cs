using System.Diagnostics;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Events;

/// <summary>
/// This shows how to process body evts.
/// </summary>
[Sample("Events", "Body Move")]
public class BodyMove : SampleBase
{
    protected const int MaxCount = 50;

    protected BodyId[] BodyIds = new BodyId[MaxCount];

    protected bool[] Sleeping = new bool[MaxCount];

    protected int Count;

    protected int SleepCount;

    protected Vec2 ExplosionPosition;

    protected float ExplosionRadius;

    protected float ExplosionMagnitude;

    public BodyMove(Settings settings)
        : base(settings)
    {
        if (Settings.Restart == false)
        {
            Global.Camera.Center = (2.0f, 8.0f);
            Global.Camera.Zoom = 25.0f * 0.55f;
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Friction = 0.1f;

            Polygon box = Geometry.MakeOffsetBox(12.0f, 0.1f, (-10.0f, -0.1f), B2Math.MakeRot(-0.15f * B2Math.Pi));
            Shape.CreatePolygonShape(groundId, shapeDef, box);

            box = Geometry.MakeOffsetBox(12.0f, 0.1f, (10.0f, -0.1f), B2Math.MakeRot(0.15f * B2Math.Pi));
            Shape.CreatePolygonShape(groundId, shapeDef, box);

            shapeDef.Restitution = 0.8f;

            box = Geometry.MakeOffsetBox(0.1f, 10.0f, (19.9f, 10.0f), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);

            box = Geometry.MakeOffsetBox(0.1f, 10.0f, (-19.9f, 10.0f), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);

            box = Geometry.MakeOffsetBox(20.0f, 0.1f, (0.0f, 20.1f), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);
        }

        SleepCount = 0;
        Count = 0;

        ExplosionPosition = (0.0f, -5.0f);
        ExplosionRadius = 10.0f;
        ExplosionMagnitude = 6.0f;
    }

    void CreateBodies()
    {
        Capsule capsule = ((-0.25f, 0.0f), (0.25f, 0.0f), 0.25f);
        Circle circle = ((0.0f, 0.0f), 0.35f);
        Polygon square = Geometry.MakeSquare(0.35f);

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;
        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();

        float x = -5.0f, y = 10.0f;
        for (int i = 0; i < 10 && Count < MaxCount; ++i)
        {
            bodyDef.Position = (x, y);
            var id = Body.CreateBody(WorldId, bodyDef);
            BodyIds[Count] = id;
            Body.SetUserData(id, id);
            Sleeping[Count] = false;

            int remainder = Count % 4;
            if (remainder == 0)
            {
                Shape.CreateCapsuleShape(BodyIds[Count], shapeDef, capsule);
            }
            else if (remainder == 1)
            {
                Shape.CreateCircleShape(BodyIds[Count], shapeDef, circle);
            }
            else if (remainder == 2)
            {
                Shape.CreatePolygonShape(BodyIds[Count], shapeDef, square);
            }
            else
            {
                Polygon poly = B2Random.Shared.RandomPolygon(0.75f);
                poly.Radius = 0.1f;
                Shape.CreatePolygonShape(BodyIds[Count], shapeDef, poly);
            }

            Count += 1;
            x += 1.0f;
        }
    }

    public override void PreStep()
    {
        if (Settings.Pause == false && (StepCount & 15) == 15 && Count < MaxCount)
        {
            CreateBodies();
        }
    }

    public override void PostStep()
    {
        // Process body evts
        BodyEvents bodyEvents = World.GetBodyEvents(WorldId);
        var events = bodyEvents.MoveEvents.Span;
        for (int i = 0; i < bodyEvents.MoveCount; ++i)
        {
            // draw the transform of every body that moved (not sleeping)
            ref BodyMoveEvent evt = ref events[i];
            Draw.DrawTransform(evt.Transform);

            // this shows a somewhat contrived way to track body sleeping
            BodyId bodyId = (BodyId)evt.UserData;
            var index = Array.IndexOf(BodyIds, bodyId);
            Debug.Assert(index > -1);
            ref bool sleeping = ref Sleeping[index];

            if (evt.FellAsleep)
            {
                sleeping = true;
                SleepCount += 1;
            }
            else
            {
                if (sleeping)
                {
                    sleeping = false;
                    SleepCount -= 1;
                }
            }
        }

        Draw.DrawCircle(ExplosionPosition, ExplosionRadius, B2HexColor.Azure);

        DrawString($"sleep count: {SleepCount}");
    }
};