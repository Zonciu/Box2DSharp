using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Bodies;

[Sample("Bodies", "Body Type")]
public class BodyTypeSample : SampleBase
{
    protected BodyId AttachmentId;

    protected BodyId SecondAttachmentId;

    protected BodyId PlatformId;

    protected BodyId SecondPayloadId;

    protected BodyId TouchingBodyId;

    protected BodyId FloatingBodyId;

    protected BodyType Type;

    protected float Speed;

    protected bool IsEnabled;

    public BodyTypeSample(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.8f, 6.4f);
            Global.Camera.Zoom = 25.0f * 0.4f;
        }

        Type = BodyType.DynamicBody;
        IsEnabled = true;

        BodyId groundId = BodyId.NullId;
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            groundId = Body.CreateBody(WorldId, bodyDef);

            Segment segment = ((-20.0f, 0.0f), (20.0f, 0.0f));
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }

        // Define attachment
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (-2.0f, 3.0f);
            AttachmentId = Body.CreateBody(WorldId, bodyDef);

            Polygon box = Geometry.MakeBox(0.5f, 2.0f);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Density = 1.0f;
            Shape.CreatePolygonShape(AttachmentId, shapeDef, box);
        }

        // Define second attachment
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = Type;
            bodyDef.IsEnabled = IsEnabled;
            bodyDef.Position = (3.0f, 3.0f);
            SecondAttachmentId = Body.CreateBody(WorldId, bodyDef);

            Polygon box = Geometry.MakeBox(0.5f, 2.0f);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Density = 1.0f;
            Shape.CreatePolygonShape(SecondAttachmentId, shapeDef, box);
        }

        // Define platform
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = Type;
            bodyDef.IsEnabled = IsEnabled;
            bodyDef.Position = (-4.0f, 5.0f);
            PlatformId = Body.CreateBody(WorldId, bodyDef);

            Polygon box = Geometry.MakeOffsetBox(0.5f, 4.0f, (4.0f, 0.0f), B2Math.MakeRot(0.5f * B2Math.Pi));

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Friction = 0.6f;
            shapeDef.Density = 2.0f;
            Shape.CreatePolygonShape(PlatformId, shapeDef, box);

            // todo joint不运动
            RevoluteJointDef revoluteDef = RevoluteJointDef.DefaultRevoluteJointDef();
            Vec2 pivot = (-2.0f, 5.0f);
            revoluteDef.BodyIdA = AttachmentId;
            revoluteDef.BodyIdB = PlatformId;
            revoluteDef.LocalAnchorA = Body.GetLocalPoint(AttachmentId, pivot);
            revoluteDef.LocalAnchorB = Body.GetLocalPoint(PlatformId, pivot);
            revoluteDef.MaxMotorTorque = 50.0f;
            revoluteDef.EnableMotor = true;
            Joint.CreateRevoluteJoint(WorldId, revoluteDef);

            pivot = (3.0f, 5.0f);
            revoluteDef.BodyIdA = SecondAttachmentId;
            revoluteDef.BodyIdB = PlatformId;
            revoluteDef.LocalAnchorA = Body.GetLocalPoint(SecondAttachmentId, pivot);
            revoluteDef.LocalAnchorB = Body.GetLocalPoint(PlatformId, pivot);
            revoluteDef.MaxMotorTorque = 50.0f;
            revoluteDef.EnableMotor = true;
            Joint.CreateRevoluteJoint(WorldId, revoluteDef);

            PrismaticJointDef prismaticDef = PrismaticJointDef.DefaultPrismaticJointDef();
            Vec2 anchor = (0.0f, 5.0f);
            prismaticDef.BodyIdA = groundId;
            prismaticDef.BodyIdB = PlatformId;
            prismaticDef.LocalAnchorA = Body.GetLocalPoint(groundId, anchor);
            prismaticDef.LocalAnchorB = Body.GetLocalPoint(PlatformId, anchor);
            prismaticDef.LocalAxisA = (1.0f, 0.0f);
            prismaticDef.MaxMotorForce = 1000.0f;
            prismaticDef.MotorSpeed = 0.0f;
            prismaticDef.EnableMotor = true;
            prismaticDef.LowerTranslation = -10.0f;
            prismaticDef.UpperTranslation = 10.0f;
            prismaticDef.EnableLimit = true;

            Joint.CreatePrismaticJoint(WorldId, prismaticDef);

            Speed = 3.0f;
        }

        // Create a payload
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (-3.0f, 8.0f);
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            Polygon box = Geometry.MakeBox(0.75f, 0.75f);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Friction = 0.6f;
            shapeDef.Density = 2.0f;

            Shape.CreatePolygonShape(bodyId, shapeDef, box);
        }

        // Create a second payload
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = Type;
            bodyDef.IsEnabled = IsEnabled;
            bodyDef.Position = (2.0f, 8.0f);
            SecondPayloadId = Body.CreateBody(WorldId, bodyDef);

            Polygon box = Geometry.MakeBox(0.75f, 0.75f);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Friction = 0.6f;
            shapeDef.Density = 2.0f;

            Shape.CreatePolygonShape(SecondPayloadId, shapeDef, box);
        }

        // Create a separate body on the ground
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = Type;
            bodyDef.IsEnabled = IsEnabled;
            bodyDef.Position = (8.0f, 0.2f);
            TouchingBodyId = Body.CreateBody(WorldId, bodyDef);

            Capsule capsule = ((0.0f, 0.0f), (1.0f, 0.0f), 0.25f);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Friction = 0.6f;
            shapeDef.Density = 2.0f;

            Shape.CreateCapsuleShape(TouchingBodyId, shapeDef, capsule);
        }

        // Create a separate floating body
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = Type;
            bodyDef.IsEnabled = IsEnabled;
            bodyDef.Position = (-8.0f, 12.0f);
            bodyDef.GravityScale = 0.0f;
            FloatingBodyId = Body.CreateBody(WorldId, bodyDef);

            Circle circle = ((0.0f, 0.5f), 0.25f);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Friction = 0.6f;
            shapeDef.Density = 2.0f;

            Shape.CreateCircleShape(FloatingBodyId, shapeDef, circle);
        }
    }

    public override void PreStep()
    {
        // Drive the kinematic body.
        if (Type == BodyType.KinematicBody)
        {
            Vec2 p = Body.GetPosition(PlatformId);
            Vec2 v = Body.GetLinearVelocity(PlatformId);

            if ((p.X < -14.0f && v.X < 0.0f) || (p.X > 6.0f && v.X > 0.0f))
            {
                v.X = -v.X;
                Body.SetLinearVelocity(PlatformId, v);
            }
        }
    }
}