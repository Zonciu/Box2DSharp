using System.Diagnostics;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Events;

[Sample("Events", "Sensor")]
public class SensorEvent : SampleBase
{
    protected const int Donut = 1;

    protected const int Human = 2;

    protected const int Count = 32;

    protected Human[] Humans = new Human [Count].Fill();

    protected Donut[] Donuts = new Donut[Count].Fill();

    protected bool[] IsSpawned = new bool[Count];

    protected int Type;

    protected float Wait;

    protected float Side;

    public SensorEvent(Settings settings)
        : base(settings)
    {
        if (Settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 0.0f);
            Global.Camera.Zoom = 25.0f * 1.333f;
        }

        Settings.DrawJoints = false;

        {
            BodyDef def = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, def);

            // Vec2 points[] = {
            //(42.333, 44.979),	(177.271, 44.979),	(177.271, 100.542), (142.875, 121.708), (177.271, 121.708),
            //(177.271, 171.979), (142.875, 193.146), (177.271, 193.146), (177.271, 222.250), (124.354, 261.938),
            //(124.354, 293.688), (95.250, 293.688),	(95.250, 261.938),	(42.333, 222.250),	(42.333, 193.146),
            //(76.729, 193.146),	(42.333, 171.979),	(42.333, 121.708),	(76.729, 121.708),	(42.333, 100.542),
            //};

            Vec2[] points =
            [
                (-16.8672504f, 31.088623f), (16.8672485f, 31.088623f), (16.8672485f, 17.1978741f),
                (8.26824951f, 11.906374f), (16.8672485f, 11.906374f), (16.8672485f, -0.661376953f),
                (8.26824951f, -5.953125f), (16.8672485f, -5.953125f), (16.8672485f, -13.229126f),
                (3.63799858f, -23.151123f), (3.63799858f, -31.088623f), (-3.63800049f, -31.088623f),
                (-3.63800049f, -23.151123f), (-16.8672504f, -13.229126f), (-16.8672504f, -5.953125f),
                (-8.26825142f, -5.953125f), (-16.8672504f, -0.661376953f), (-16.8672504f, 11.906374f),
                (-8.26825142f, 11.906374f), (-16.8672504f, 17.1978741f),
            ];

            int count = points.Length;

            // float scale = 0.25f;
            // Vec2 lower = (FLT_MAX, FLT_MAX);
            // Vec2 upper = (-FLT_MAX, -FLT_MAX);
            // for (int i = 0; i < count; ++i)
            //{
            //	points[i].X = scale * points[i].X;
            //	points[i].Y = -scale * points[i].Y;

            //	lower = Min(lower, points[i]);
            //	upper = Max(upper, points[i]);
            //}

            // Vec2 center = MulSV(0.5f, Add(lower, upper));
            // for (int i = 0; i < count; ++i)
            //{
            //	points[i] = Sub(points[i], center);
            // }

            // for (int i = 0; i < count / 2; ++i)
            //{
            //	Vec2 temp = points[i];
            //	points[i] = points[count - 1 - i];
            //	points[count - 1 - i] = temp;
            // }

            // printf("{");
            // for (int i = 0; i < count; ++i)
            //{
            //	printf("(%.9g, %.9g),", points[i].X, points[i].Y);
            // }
            // printf("};\n");

            ChainDef chainDef = ChainDef.DefaultChainDef();
            chainDef.Points = points;
            chainDef.Count = count;
            chainDef.IsLoop = true;
            chainDef.Friction = 0.2f;
            Shape.CreateChain(groundId, chainDef);

            float sign = 1.0f;
            float y = 14.0f;
            for (int i = 0; i < 3; ++i)
            {
                BodyDef bodyDef = BodyDef.DefaultBodyDef();
                bodyDef.Position = (0.0f, y);
                bodyDef.Type = BodyType.DynamicBody;

                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

                Polygon box = Geometry.MakeBox(6.0f, 0.5f);
                ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
                shapeDef.Friction = 0.1f;
                shapeDef.Restitution = 1.0f;
                shapeDef.Density = 1.0f;

                Shape.CreatePolygonShape(bodyId, shapeDef, box);

                RevoluteJointDef revoluteDef = RevoluteJointDef.DefaultRevoluteJointDef();
                revoluteDef.BodyIdA = groundId;
                revoluteDef.BodyIdB = bodyId;
                revoluteDef.LocalAnchorA = bodyDef.Position;
                revoluteDef.LocalAnchorB = Vec2.Zero;
                revoluteDef.MaxMotorTorque = 200.0f;
                revoluteDef.MotorSpeed = 2.0f * sign;
                revoluteDef.EnableMotor = true;

                Joint.CreateRevoluteJoint(WorldId, revoluteDef);

                y -= 14.0f;
                sign = -sign;
            }

            {
                Polygon box = Geometry.MakeOffsetBox(4.0f, 1.0f, (0.0f, -30.5f), Rot.Identity);
                ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
                shapeDef.IsSensor = true;
                Shape.CreatePolygonShape(groundId, shapeDef, box);
            }
        }

        Wait = 0.5f;
        Side = -15.0f;
        Type = Human;

        for (int i = 0; i < Count; ++i)
        {
            IsSpawned[i] = false;
        }

        CreateElement();
    }

    void CreateElement()
    {
        int index = -1;
        for (int i = 0; i < Count; ++i)
        {
            if (IsSpawned[i] == false)
            {
                index = i;
                break;
            }
        }

        if (index == -1)
        {
            return;
        }

        Vec2 center = (Side, 29.5f);

        if (Type == Donut)
        {
            Donut donut = Donuts[index];

            // donut.Spawn(WorldId, center, index + 1, donut);
            donut.Spawn(WorldId, center, 1.0f, 0, donut);
        }
        else
        {
            Human human = Humans[index];
            human.Spawn(WorldId, center, 2.0f, 0.05f, 0.0f, 0.0f, index + 1, human, false);
        }

        IsSpawned[index] = true;
        Side = -Side;
    }

    void DestroyElement(int index)
    {
        if (Type == Donut)
        {
            Donut donut = Donuts[index];
            donut.Despawn();
        }
        else
        {
            Human human = Humans[index];
            human.Despawn();
        }

        IsSpawned[index] = false;
    }

    protected void Clear()
    {
        for (int i = 0; i < Count; ++i)
        {
            if (IsSpawned[i] == true)
            {
                if (Type == Donut)
                {
                    Donuts[i].Despawn();
                }
                else
                {
                    Humans[i].Despawn();
                }

                IsSpawned[i] = false;
            }
        }
    }

    public override void Step()
    {
        if (StepCount == 832)
        {
            StepCount += 0;
        }

        base.Step();

        // Discover rings that touch the bottom sensor
        bool[] deferredDestructions = new bool[Count];
        SensorEvents sensorEvents = World.GetSensorEvents(WorldId);
        var beginEvents = sensorEvents.BeginEvents.Span;
        for (int i = 0; i < sensorEvents.BeginCount; ++i)
        {
            ref readonly SensorBeginTouchEvent evt = ref beginEvents[i];
            ShapeId visitorId = evt.VisitorShapeId;
            BodyId bodyId = Shape.GetBody(visitorId);

            if (Type == Donut)
            {
                var donut = (Donut?)Body.GetUserData(bodyId);
                if (donut != null)
                {
                    // 获取当前donut对象在m_donuts的序号
                    var index = Array.IndexOf(Donuts, donut);
                    Debug.Assert(0 <= index && index < Count);

                    // Defer destruction to avoid double destruction and evt invalidation (orphaned shape ids)
                    deferredDestructions[index] = true;
                }
            }
            else
            {
                var human = (Human?)Body.GetUserData(bodyId);
                if (human != null)
                {
                    var index = Array.IndexOf(Humans, human);
                    Debug.Assert(0 <= index && index < Count);

                    // Defer destruction to avoid double destruction and evt invalidation (orphaned shape ids)
                    deferredDestructions[index] = true;
                }
            }
        }

        // todo destroy mouse joint if necessary

        // Safely destroy rings that hit the bottom sensor
        for (int i = 0; i < Count; ++i)
        {
            if (deferredDestructions[i])
            {
                DestroyElement(i);
            }
        }

        if (Settings.Hertz > 0.0f && Settings.Pause == false)
        {
            Wait -= 1.0f / Settings.Hertz;
            if (Wait < 0.0f)
            {
                CreateElement();
                Wait += 0.5f;
            }
        }
    }
}