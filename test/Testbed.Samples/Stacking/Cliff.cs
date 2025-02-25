using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Stacking;

[Sample("Stacking", "Cliff")]
public class Cliff : SampleBase
{
    public Cliff(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Zoom = 25.0f * 0.5f;
            Global.Camera.Center = (0.0f, 5.0f);
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Position = (0.0f, 0.0f);
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Polygon box = Geometry.MakeOffsetBox(100.0f, 1.0f, (0.0f, -1.0f), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);

            Segment segment = ((-14.0f, 4.0f), (-8.0f, 4.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);

            box = Geometry.MakeOffsetBox(3.0f, 0.5f, (0.0f, 4.0f), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);

            Capsule capsule = ((8.5f, 4.0f), (13.5f, 4.0f), 0.5f);
            Shape.CreateCapsuleShape(groundId, shapeDef, capsule);
        }

        Flip = false;

        for (int i = 0; i < 9; ++i)
        {
            BodyIds[i] = BodyId.NullId;
        }

        CreateBodies();
    }

    protected void CreateBodies()
    {
        for (int i = 0; i < 9; ++i)
        {
            if (BodyIds[i].IsNotNull)
            {
                Body.DestroyBody(BodyIds[i]);
                BodyIds[i] = BodyId.NullId;
            }
        }

        float sign = Flip ? -1.0f : 1.0f;

        Capsule capsule = ((-0.25f, 0.0f), (0.25f, 0.0f), 0.25f);
        Circle circle = ((0.0f, 0.0f), 0.5f);
        Polygon square = Geometry.MakeSquare(0.5f);

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;

        {
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Friction = 0.01f;
            bodyDef.LinearVelocity = (2.0f * sign, 0.0f);

            float offset = Flip ? -4.0f : 0.0f;

            bodyDef.Position = (-9.0f + offset, 4.25f);
            BodyIds[0] = Body.CreateBody(WorldId, bodyDef);
            Shape.CreateCapsuleShape(BodyIds[0], shapeDef, capsule);

            bodyDef.Position = (2.0f + offset, 4.75f);
            BodyIds[1] = Body.CreateBody(WorldId, bodyDef);
            Shape.CreateCapsuleShape(BodyIds[1], shapeDef, capsule);

            bodyDef.Position = (13.0f + offset, 4.75f);
            BodyIds[2] = Body.CreateBody(WorldId, bodyDef);
            Shape.CreateCapsuleShape(BodyIds[2], shapeDef, capsule);
        }

        {
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Friction = 0.01f;
            bodyDef.LinearVelocity = (2.5f * sign, 0.0f);

            bodyDef.Position = (-11.0f, 4.5f);
            BodyIds[3] = Body.CreateBody(WorldId, bodyDef);
            Shape.CreatePolygonShape(BodyIds[3], shapeDef, square);

            bodyDef.Position = (0.0f, 5.0f);
            BodyIds[4] = Body.CreateBody(WorldId, bodyDef);
            Shape.CreatePolygonShape(BodyIds[4], shapeDef, square);

            bodyDef.Position = (11.0f, 5.0f);
            BodyIds[5] = Body.CreateBody(WorldId, bodyDef);
            Shape.CreatePolygonShape(BodyIds[5], shapeDef, square);
        }

        {
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Friction = 0.2f;
            bodyDef.LinearVelocity = (1.5f * sign, 0.0f);

            float offset = Flip ? 4.0f : 0.0f;

            bodyDef.Position = (-13.0f + offset, 4.5f);
            BodyIds[6] = Body.CreateBody(WorldId, bodyDef);
            Shape.CreateCircleShape(BodyIds[6], shapeDef, circle);

            bodyDef.Position = (-2.0f + offset, 5.0f);
            BodyIds[7] = Body.CreateBody(WorldId, bodyDef);
            Shape.CreateCircleShape(BodyIds[7], shapeDef, circle);

            bodyDef.Position = (9.0f + offset, 5.0f);
            BodyIds[8] = Body.CreateBody(WorldId, bodyDef);
            Shape.CreateCircleShape(BodyIds[8], shapeDef, circle);
        }
    }

    protected BodyId[] BodyIds = new BodyId[9];

    protected bool Flip;
}