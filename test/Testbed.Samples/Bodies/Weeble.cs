using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Bodies;

[Sample("Bodies", "Weeble")]
public class Weeble : SampleBase
{
    protected BodyId WeebleId;

    protected Vec2 ExplosionPosition;

    protected float ExplosionRadius;

    protected float ExplosionMagnitude;

    public Weeble(Settings settings)
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

        // Build weeble
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (0.0f, 3.0f);
            bodyDef.Rotation = B2Math.MakeRot(0.25f * B2Math.Pi);
            WeebleId = Body.CreateBody(WorldId, bodyDef);

            Capsule capsule = ((0.0f, -1.0f), (0.0f, 1.0f), 1.0f);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Density = 1.0f;
            Shape.CreateCapsuleShape(WeebleId, shapeDef, capsule);

            float mass = Body.GetMass(WeebleId);
            float inertiaTensor = Body.GetRotationalInertia(WeebleId);

            float offset = 1.5f;

            // See: https://en.wikipedia.org/wiki/Parallel_axis_theorem
            inertiaTensor += mass * offset * offset;

            MassData massData = (mass, (0.0f, -offset), inertiaTensor);
            Body.SetMassData(WeebleId, massData);
        }

        ExplosionPosition = (0.0f, 0.0f);
        ExplosionRadius = 2.0f;
        ExplosionMagnitude = 8.0f;
    }

    public override void PostStep()
    {
        Draw.DrawCircle(ExplosionPosition, ExplosionRadius, B2HexColor.Azure);
    }
}