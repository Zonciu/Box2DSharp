using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Joints;

[Sample("Joints", "Doohickey Farm")]
public class DoohickeyFarm : SampleBase
{
    public DoohickeyFarm(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 5.0f);
            Global.Camera.Zoom = 25.0f * 0.35f;
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Segment segment = ((-20.0f, 0.0f), (20.0f, 0.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);

            Polygon box = Geometry.MakeOffsetBox(1.0f, 1.0f, (0.0f, 1.0f), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);
        }

        float y = 4.0f;
        for (int i = 0; i < 4; ++i)
        {
            Doohickey doohickey = new();
            doohickey.Spawn(WorldId, (0.0f, y), 0.5f);
            y += 2.0f;
        }
    }
}