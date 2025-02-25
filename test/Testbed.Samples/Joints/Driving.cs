using System.Diagnostics;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Joints;

[Sample("Joints", "Driving")]
public class Driving : SampleBase
{
    public Driving(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center.Y = 5.0f;
            Global.Camera.Zoom = 25.0f * 0.4f;
            settings.DrawJoints = false;
        }

        BodyId groundId;
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            groundId = Body.CreateBody(WorldId, bodyDef);

            Vec2[] points = new Vec2[25];
            int count = 24;

            // fill in reverse to match line list convention
            points[count--] = (-20.0f, -20.0f);
            points[count--] = (-20.0f, 0.0f);
            points[count--] = (20.0f, 0.0f);

            float[] hs = [0.25f, 1.0f, 4.0f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f];
            float x = 20.0f, y1 = 0.0f, dx = 5.0f;

            for (int j = 0; j < 2; ++j)
            {
                for (int i = 0; i < 10; ++i)
                {
                    float y2 = hs[i];
                    points[count--] = (x + dx, y2);
                    y1 = y2;
                    x += dx;
                }
            }

            // flat before bridge
            points[count--] = (x + 40.0f, 0.0f);
            points[count--] = (x + 40.0f, -20.0f);

            Debug.Assert(count == -1);

            ChainDef chainDef = ChainDef.DefaultChainDef();
            chainDef.Points = points;
            chainDef.Count = 25;
            chainDef.IsLoop = true;
            Shape.CreateChain(groundId, chainDef);

            // flat after bridge
            x += 80.0f;
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Segment segment = ((x, 0.0f), (x + 40.0f, 0.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);

            // jump ramp
            x += 40.0f;
            segment = ((x, 0.0f), (x + 10.0f, 5.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);

            // final corner
            x += 20.0f;
            segment = ((x, 0.0f), (x + 40.0f, 0.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);

            x += 40.0f;
            segment = ((x, 0.0f), (x, 20.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }

        // Teeter
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Position = (140.0f, 1.0f);
            bodyDef.AngularVelocity = 1.0f;
            bodyDef.Type = BodyType.DynamicBody;
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Polygon box = Geometry.MakeBox(10.0f, 0.25f);
            Shape.CreatePolygonShape(bodyId, shapeDef, box);

            Vec2 pivot = bodyDef.Position;
            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
            jointDef.BodyIdA = groundId;
            jointDef.BodyIdB = bodyId;
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.LowerAngle = -8.0f * B2Math.Pi / 180.0f;
            jointDef.UpperAngle = 8.0f * B2Math.Pi / 180.0f;
            jointDef.EnableLimit = true;
            Joint.CreateRevoluteJoint(WorldId, jointDef);
        }

        // Bridge
        {
            int N = 20;
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Capsule capsule = ((-1.0f, 0.0f), (1.0f, 0.0f), 0.125f);

            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();

            BodyId prevBodyId = groundId;
            for (int i = 0; i < N; ++i)
            {
                BodyDef bodyDef = BodyDef.DefaultBodyDef();
                bodyDef.Type = BodyType.DynamicBody;
                bodyDef.Position = (161.0f + 2.0f * i, -0.125f);
                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
                Shape.CreateCapsuleShape(bodyId, shapeDef, capsule);

                Vec2 pivot = (160.0f + 2.0f * i, -0.125f);
                jointDef.BodyIdA = prevBodyId;
                jointDef.BodyIdB = bodyId;
                jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
                jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
                Joint.CreateRevoluteJoint(WorldId, jointDef);

                prevBodyId = bodyId;
            }

            {
                Vec2 pivot = (160.0f + 2.0f * N, -0.125f);
                jointDef.BodyIdA = prevBodyId;
                jointDef.BodyIdB = groundId;
                jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
                jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
                jointDef.EnableMotor = true;
                jointDef.MaxMotorTorque = 50.0f;
                Joint.CreateRevoluteJoint(WorldId, jointDef);
            }
        }

        // Boxes
        {
            Polygon box = Geometry.MakeBox(0.5f, 0.5f);

            BodyId bodyId;
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Friction = 0.25f;
            shapeDef.Restitution = 0.25f;
            shapeDef.Density = 0.25f;

            bodyDef.Position = (230.0f, 0.5f);
            bodyId = Body.CreateBody(WorldId, bodyDef);
            Shape.CreatePolygonShape(bodyId, shapeDef, box);

            bodyDef.Position = (230.0f, 1.5f);
            bodyId = Body.CreateBody(WorldId, bodyDef);
            Shape.CreatePolygonShape(bodyId, shapeDef, box);

            bodyDef.Position = (230.0f, 2.5f);
            bodyId = Body.CreateBody(WorldId, bodyDef);
            Shape.CreatePolygonShape(bodyId, shapeDef, box);

            bodyDef.Position = (230.0f, 3.5f);
            bodyId = Body.CreateBody(WorldId, bodyDef);
            Shape.CreatePolygonShape(bodyId, shapeDef, box);

            bodyDef.Position = (230.0f, 4.5f);
            bodyId = Body.CreateBody(WorldId, bodyDef);
            Shape.CreatePolygonShape(bodyId, shapeDef, box);
        }

        // Car

        m_throttle = 0.0f;
        m_speed = 35.0f;
        m_torque = 2.5f;
        m_hertz = 5.0f;
        m_dampingRatio = 0.7f;

        m_car.Spawn(WorldId, (0.0f, 0.0f), 1.0f, m_hertz, m_dampingRatio, m_torque, null);
    }

    public override void OnKeyDown(KeyInputEventArgs keyInput)
    {
        base.OnKeyDown(keyInput);
        switch (keyInput.Key)
        {
        case KeyCodes.A:
            m_throttle = 1.0f;
            m_car.SetSpeed(m_speed);
            break;
        case KeyCodes.S:
            m_throttle = 0.0f;
            m_car.SetSpeed(0.0f);
            break;
        case KeyCodes.D:
            m_throttle = -1.0f;
            m_car.SetSpeed(-m_speed);
            break;
        }
    }

    protected override void OnRender()
    {
        DrawString("Keys: left = a, brake = s, right = d");

        Vec2 linearVelocity = Body.GetLinearVelocity(m_car.ChassisId);
        float kph = linearVelocity.X * 3.6f;
        DrawString($"speed in kph: {kph:F2}");

        Vec2 carPosition = Body.GetPosition(m_car.ChassisId);
        Global.Camera.Center.X = carPosition.X;
    }

    protected Car m_car = new();

    protected float m_throttle;

    protected float m_hertz;

    protected float m_dampingRatio;

    protected float m_torque;

    protected float m_speed;
}