using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Continuous;

/// <summary>
/// Speculative collision failure case suggested by Dirk Gregorius. This uses
/// a simple fallback scheme to prevent tunneling.
/// </summary>
[Sample("Continuous", "Speculative Fallback")]
public class SpeculativeFallback : SampleBase
{
    public SpeculativeFallback(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (1.0f, 5.0f);
            Global.Camera.Zoom = 25.0f * 0.25f;
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Segment segment = ((-10.0f, 0.0f), (10.0f, 0.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);

            Vec2[] points = [(-2.0f, 4.0f), (2.0f, 4.0f), (2.0f, 4.1f), (-0.5f, 4.2f), (-2.0f, 4.2f)];
            Hull hull = HullFunc.ComputeHull(points, 5);
            Polygon poly = Geometry.MakePolygon(hull, 0.0f);
            Shape.CreatePolygonShape(groundId, shapeDef, poly);
        }

        // Fast moving skinny box. Also testing a large shape offset.
        {
            float offset = 8.0f;
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (offset, 12.0f);
            bodyDef.LinearVelocity = (0.0f, -100.0f);
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Polygon box = Geometry.MakeOffsetBox(2.0f, 0.05f, (-offset, 0.0f), B2Math.MakeRot(B2Math.Pi));
            Shape.CreatePolygonShape(bodyId, shapeDef, box);
        }
    }
}