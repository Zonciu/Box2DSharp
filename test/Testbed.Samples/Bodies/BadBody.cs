using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Bodies;

[Sample("Bodies", "BadBody")]
public class BadBody : SampleBase
{
    private BodyId _badBodyId;

    public BadBody(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (2.3f, 10.0f);
            Global.Camera.Zoom = 25.0f * 0.5f;
        }

        BodyId groundId = BodyId.NullId;
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            groundId = Body.CreateBody(WorldId, bodyDef);

            Segment segment = ((-20.0f, 0.0f), (20.0f, 0.0f));
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }

        // Build a bad body
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (0.0f, 3.0f);
            bodyDef.AngularVelocity = 0.2f;
            bodyDef.Rotation = B2Math.MakeRot(0.25f * B2Math.Pi);

            _badBodyId = Body.CreateBody(WorldId, bodyDef);

            Capsule capsule = ((0.0f, -1.0f), (0.0f, 1.0f), 1.0f);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();

            // density set to zero intentionally to create a bad body
            shapeDef.Density = 0.0f;
            Shape.CreateCapsuleShape(_badBodyId, shapeDef, capsule);
        }

        // Build a normal body
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (2.0f, 3.0f);
            bodyDef.Rotation = B2Math.MakeRot(0.25f * B2Math.Pi);

            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            Capsule capsule = ((0.0f, -1.0f), (0.0f, 1.0f), 1.0f);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();

            Shape.CreateCapsuleShape(bodyId, shapeDef, capsule);
        }
    }

    public override void PostStep()
    {
        DrawString("A bad body is a dynamic body with no mass and behaves like a kinematic body.");
        DrawString("Bad bodies are considered invalid and a user bug. Behavior is not guaranteed.");

        // For science
        Body.ApplyForceToCenter(_badBodyId, (0.0f, 10.0f), true);
    }
}