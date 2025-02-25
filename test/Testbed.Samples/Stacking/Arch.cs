using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Stacking;

[Sample("Stacking", "Arch")]
public class Arch : SampleBase
{
    public Arch(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 8.0f);
            Global.Camera.Zoom = 25.0f * 0.35f;
        }

        Vec2[] ps1 =
        [
            (16.0f, 0.0f),
            (14.93803712795643f, 5.133601056842984f),
            (13.79871746027416f, 10.24928069555078f),
            (12.56252963284711f, 15.34107019122473f),
            (11.20040987372525f, 20.39856541571217f),
            (9.66521217819836f, 25.40369899225096f),
            (7.87179930638133f, 30.3179337000085f),
            (5.635199558196225f, 35.03820717801641f),
            (2.405937953536585f, 39.09554102558315f)
        ];

        Vec2[] ps2 =
        [
            (24.0f, 0.0f),
            (22.33619528222415f, 6.02299846205841f),
            (20.54936888969905f, 12.00964361211476f),
            (18.60854610798073f, 17.9470321677465f),
            (16.46769273811807f, 23.81367936585418f),
            (14.05325025774858f, 29.57079353071012f),
            (11.23551045834022f, 35.13775818285372f),
            (7.752568160730571f, 40.30450679009583f),
            (3.016931552701656f, 44.28891593799322f)
        ];

        float scale = 0.25f;
        for (int i = 0; i < 9; ++i)
        {
            ps1[i] = B2Math.MulSV(scale, ps1[i]);
            ps2[i] = B2Math.MulSV(scale, ps2[i]);
        }

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Friction = 0.6f;

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);
            Segment segment = ((-100.0f, 0.0f), (100.0f, 0.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;

            for (int i = 0; i < 8; ++i)
            {
                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
                Vec2[] ps = [ps1[i], ps2[i], ps2[i + 1], ps1[i + 1]];
                Hull hull = HullFunc.ComputeHull(ps, 4);
                Polygon polygon = Geometry.MakePolygon(hull, 0.0f);
                Shape.CreatePolygonShape(bodyId, shapeDef, polygon);
            }

            for (int i = 0; i < 8; ++i)
            {
                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
                Vec2[] ps =
                [
                    (-ps2[i].X, ps2[i].Y),
                    (-ps1[i].X, ps1[i].Y),
                    (-ps1[i + 1].X, ps1[i + 1].Y),
                    (-ps2[i + 1].X, ps2[i + 1].Y)
                ];
                Hull hull = HullFunc.ComputeHull(ps, 4);
                Polygon polygon = Geometry.MakePolygon(hull, 0.0f);
                Shape.CreatePolygonShape(bodyId, shapeDef, polygon);
            }

            {
                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
                Vec2[] ps = [ps1[8], ps2[8], (-ps2[8].X, ps2[8].Y), (-ps1[8].X, ps1[8].Y)];
                Hull hull = HullFunc.ComputeHull(ps, 4);
                Polygon polygon = Geometry.MakePolygon(hull, 0.0f);
                Shape.CreatePolygonShape(bodyId, shapeDef, polygon);
            }

            for (int i = 0; i < 4; ++i)
            {
                Polygon box = Geometry.MakeBox(2.0f, 0.5f);
                bodyDef.Position = (0.0f, 0.5f + ps2[8].Y + 1.0f * i);
                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
                Shape.CreatePolygonShape(bodyId, shapeDef, box);
            }
        }
    }
}