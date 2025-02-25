using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Bodies;

[Sample("Bodies", "Sleep")]
public class Sleep : SampleBase
{
    protected BodyId PendulumId;

    protected ShapeId GroundShapeId;

    protected ShapeId[] SensorIds = new ShapeId[2];

    protected bool[] SensorTouching = new bool[2];

    public Sleep(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (3.0f, 50.0f);
            Global.Camera.Zoom = 25.0f * 2.2f;
        }

        BodyId groundId = BodyId.NullId;
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            groundId = Body.CreateBody(WorldId, bodyDef);

            Segment segment = ((-20.0f, 0.0f), (20.0f, 0.0f));
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            GroundShapeId = Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }

        // Sleeping body with sensors
        for (int i = 0; i < 2; ++i)
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (-4.0f, 3.0f + 2.0f * i)
                ;
            bodyDef.IsAwake = false;
            bodyDef.EnableSleep = true;
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            Capsule capsule = ((0.0f, 1.0f), (1.0f, 1.0f), 0.75f);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreateCapsuleShape(bodyId, shapeDef, capsule);

            shapeDef.IsSensor = true;
            capsule.Radius = 1.0f;
            SensorIds[i] = Shape.CreateCapsuleShape(bodyId, shapeDef, capsule);
            SensorTouching[i] = false;
        }

        // Sleeping body but sleep is disabled
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (0.0f, 3.0f);
            bodyDef.IsAwake = false;
            bodyDef.EnableSleep = false;
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            Circle circle = ((1.0f, 1.0f), 1.0f);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreateCircleShape(bodyId, shapeDef, circle);
        }

        // Awake body and sleep is disabled
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (5.0f, 3.0f)
                ;
            bodyDef.IsAwake = true;
            bodyDef.EnableSleep = false;
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            Polygon box = Geometry.MakeOffsetBox(1.0f, 1.0f, (0.0f, 1.0f), B2Math.MakeRot(0.25f * B2Math.Pi));
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreatePolygonShape(bodyId, shapeDef, box);
        }

        // A sleeping body to test waking on collision
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (5.0f, 1.0f)
                ;
            bodyDef.IsAwake = false;
            bodyDef.EnableSleep = true;
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            Polygon box = Geometry.MakeSquare(1.0f);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreatePolygonShape(bodyId, shapeDef, box);
        }

        // A long pendulum
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (0.0f, 100.0f)
                ;
            bodyDef.AngularDamping = 0.5f;
            bodyDef.SleepThreshold = 0.05f;
            PendulumId = Body.CreateBody(WorldId, bodyDef);

            Capsule capsule = ((0.0f, 0.0f), (90.0f, 0.0f), 0.25f);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreateCapsuleShape(PendulumId, shapeDef, capsule);

            Vec2 pivot = bodyDef.Position;
            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
            jointDef.BodyIdA = groundId;
            jointDef.BodyIdB = PendulumId;
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            Joint.CreateRevoluteJoint(WorldId, jointDef);
        }
    }

    public override void PostStep()
    {
        // Detect sensors touching the ground
        SensorEvents sensorEvents = World.GetSensorEvents(WorldId);

        for (int i = 0; i < sensorEvents.BeginCount; ++i)
        {
            SensorBeginTouchEvent evt = sensorEvents.BeginEvents.Span[i];
            if (evt.VisitorShapeId == GroundShapeId)
            {
                if (evt.SensorShapeId == SensorIds[0])
                {
                    SensorTouching[0] = true;
                }
                else if (evt.SensorShapeId == SensorIds[1])
                {
                    SensorTouching[1] = true;
                }
            }
        }

        for (int i = 0; i < sensorEvents.EndCount; ++i)
        {
            SensorEndTouchEvent evt = sensorEvents.EndEvents.Span[i];
            if (evt.VisitorShapeId == GroundShapeId)
            {
                if (evt.SensorShapeId == SensorIds[0])
                {
                    SensorTouching[0] = false;
                }
                else if (evt.SensorShapeId == SensorIds[1])
                {
                    SensorTouching[1] = false;
                }
            }
        }

        for (int i = 0; i < 2; ++i)
        {
            DrawString($"sensor touch {i} = {(SensorTouching[i] ? "true" : "false")}");
        }
    }
}