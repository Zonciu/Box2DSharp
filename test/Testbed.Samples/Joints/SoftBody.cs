using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Joints;

[Sample("Joints", "Soft Body")]
public class SoftBody : SampleBase
{
    public SoftBody(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 5.0f);
            Global.Camera.Zoom = 25.0f * 0.25f;
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Segment segment = ((-20.0f, 0.0f), (20.0f, 0.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }

        m_donut.Spawn(WorldId, (0.0f, 10.0f), 2.0f, 0, null);
    }

    protected Donut m_donut = new();
}