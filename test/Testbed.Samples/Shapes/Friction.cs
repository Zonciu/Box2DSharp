using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Shapes;

[Sample("Shapes", "Friction")]
public class Friction : SampleBase
{
    public Friction(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 14.0f);
            Global.Camera.Zoom = 25.0f * 0.6f;
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Friction = 0.2f;

            Segment segment = ((-40.0f, 0.0f), (40.0f, 0.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);

            Polygon box = Geometry.MakeOffsetBox(13.0f, 0.25f, (-4.0f, 22.0f), B2Math.MakeRot(-0.25f));
            Shape.CreatePolygonShape(groundId, shapeDef, box);

            box = Geometry.MakeOffsetBox(0.25f, 1.0f, (10.5f, 19.0f), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);

            box = Geometry.MakeOffsetBox(13.0f, 0.25f, (4.0f, 14.0f), B2Math.MakeRot(0.25f));
            Shape.CreatePolygonShape(groundId, shapeDef, box);

            box = Geometry.MakeOffsetBox(0.25f, 1.0f, (-10.5f, 11.0f), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);

            box = Geometry.MakeOffsetBox(13.0f, 0.25f, (-4.0f, 6.0f), B2Math.MakeRot(-0.25f));
            Shape.CreatePolygonShape(groundId, shapeDef, box);
        }

        {
            Polygon box = Geometry.MakeBox(0.5f, 0.5f);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Density = 25.0f;

            float[] friction = [0.75f, 0.5f, 0.35f, 0.1f, 0.0f];

            for (int i = 0; i < 5; ++i)
            {
                BodyDef bodyDef = BodyDef.DefaultBodyDef();
                bodyDef.Type = BodyType.DynamicBody;
                bodyDef.Position = (-15.0f + 4.0f * i, 28.0f);
                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

                shapeDef.Friction = friction[i];
                Shape.CreatePolygonShape(bodyId, shapeDef, box);
            }
        }
    }
}