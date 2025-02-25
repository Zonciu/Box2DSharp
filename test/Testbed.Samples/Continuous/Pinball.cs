using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Continuous;

[Sample("Continuous", "Pin Ball")]
public class Pinball : SampleBase
{
    // This shows a fast moving body that uses continuous collision versus static and dynamic bodies.
    // This is achieved by setting the ball body as a *bullet*.
    JointId m_leftJointId;

    JointId m_rightJointId;

    BodyId m_ballId;

    public Pinball(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 9.0f);
            Global.Camera.Zoom = 25.0f * 0.5f;
        }

        settings.DrawJoints = false;

        // Ground body
        BodyId groundId = new();
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            groundId = Body.CreateBody(WorldId, bodyDef);

            Vec2[] vs = [(-8.0f, 6.0f), (-8.0f, 20.0f), (8.0f, 20.0f), (8.0f, 6.0f), (0.0f, -2.0f)];

            ChainDef chainDef = ChainDef.DefaultChainDef();
            chainDef.Points = vs;
            chainDef.Count = 5;
            chainDef.IsLoop = true;
            Shape.CreateChain(groundId, chainDef);
        }

        // Flippers
        {
            Vec2 p1 = (-2.0f, 0.0f), p2 = (2.0f, 0.0f);

            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.EnableSleep = false;

            bodyDef.Position = p1;
            BodyId leftFlipperId = Body.CreateBody(WorldId, bodyDef);

            bodyDef.Position = p2;
            BodyId rightFlipperId = Body.CreateBody(WorldId, bodyDef);

            Polygon box = Geometry.MakeBox(1.75f, 0.2f);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();

            Shape.CreatePolygonShape(leftFlipperId, shapeDef, box);
            Shape.CreatePolygonShape(rightFlipperId, shapeDef, box);

            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
            jointDef.BodyIdA = groundId;
            jointDef.LocalAnchorB = Vec2.Zero;
            jointDef.EnableMotor = true;
            jointDef.MaxMotorTorque = 1000.0f;
            jointDef.EnableLimit = true;

            jointDef.MotorSpeed = 0.0f;
            jointDef.LocalAnchorA = p1;
            jointDef.BodyIdB = leftFlipperId;
            jointDef.LowerAngle = -30.0f * B2Math.Pi / 180.0f;
            jointDef.UpperAngle = 5.0f * B2Math.Pi / 180.0f;
            m_leftJointId = Joint.CreateRevoluteJoint(WorldId, jointDef);

            jointDef.MotorSpeed = 0.0f;
            jointDef.LocalAnchorA = p2;
            jointDef.BodyIdB = rightFlipperId;
            jointDef.LowerAngle = -5.0f * B2Math.Pi / 180.0f;
            jointDef.UpperAngle = 30.0f * B2Math.Pi / 180.0f;
            m_rightJointId = Joint.CreateRevoluteJoint(WorldId, jointDef);
        }

        // Spinners
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (-4.0f, 17.0f);

            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Polygon box1 = Geometry.MakeBox(1.5f, 0.125f);
            Polygon box2 = Geometry.MakeBox(0.125f, 1.5f);

            Shape.CreatePolygonShape(bodyId, shapeDef, box1);
            Shape.CreatePolygonShape(bodyId, shapeDef, box2);

            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
            jointDef.BodyIdA = groundId;
            jointDef.BodyIdB = bodyId;
            jointDef.LocalAnchorA = bodyDef.Position;
            jointDef.LocalAnchorB = Vec2.Zero;
            jointDef.EnableMotor = true;
            jointDef.MaxMotorTorque = 0.1f;
            Joint.CreateRevoluteJoint(WorldId, jointDef);

            bodyDef.Position = (4.0f, 8.0f);
            bodyId = Body.CreateBody(WorldId, bodyDef);
            Shape.CreatePolygonShape(bodyId, shapeDef, box1);
            Shape.CreatePolygonShape(bodyId, shapeDef, box2);
            jointDef.LocalAnchorA = bodyDef.Position;
            jointDef.BodyIdB = bodyId;
            Joint.CreateRevoluteJoint(WorldId, jointDef);
        }

        // Bumpers
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Position = (-4.0f, 8.0f);

            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Restitution = 1.5f;

            Circle circle = ((0.0f, 0.0f), 1.0f);
            Shape.CreateCircleShape(bodyId, shapeDef, circle);

            bodyDef.Position = (4.0f, 17.0f);
            bodyId = Body.CreateBody(WorldId, bodyDef);
            Shape.CreateCircleShape(bodyId, shapeDef, circle);
        }

        // Ball
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Position = (1.0f, 15.0f);
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.IsBullet = true;

            m_ballId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Circle circle = ((0.0f, 0.0f), 0.2f);
            Shape.CreateCircleShape(m_ballId, shapeDef, circle);
        }
    }

    public override void PostStep()
    {
        base.PostStep();
        if (Input.IsKeyDown(KeyCodes.Space))
        {
            RevoluteJointFunc.SetMotorSpeed(m_leftJointId, 20.0f);
            RevoluteJointFunc.SetMotorSpeed(m_rightJointId, -20.0f);
        }
        else
        {
            RevoluteJointFunc.SetMotorSpeed(m_leftJointId, -10.0f);
            RevoluteJointFunc.SetMotorSpeed(m_rightJointId, 10.0f);
        }
    }
}