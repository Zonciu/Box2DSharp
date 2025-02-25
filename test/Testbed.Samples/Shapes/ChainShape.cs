using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Shapes;

[Sample("Shapes", "Chain Shape")]
public class ChainShape : SampleBase
{
    public const int CircleShape = 0;

    public const int CapsuleShape = 1;

    public const int BoxShape = 2;

    public ChainShape(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 0.0f);
            Global.Camera.Zoom = 25.0f * 1.75f;
        }

        GroundId = BodyId.NullId;
        BodyId = BodyId.NullId;
        ChainId = ChainId.NullId;
        ShapeId = ShapeId.NullId;
        ShapeType = CircleShape;
        Restitution = 0.0f;
        Friction = 0.2f;
        CreateScene();
        Launch();
    }

    void CreateScene()
    {
        if (GroundId.IsNotNull)
        {
            Body.DestroyBody(GroundId);
        }

        // https://betravis.Github.Io/shape-tools/path-to-polygon/
        // Vec2 points[] = {(-20.58325, 14.54175), (-21.90625, 15.8645),		 (-24.552, 17.1875),
        //				   (-27.198, 11.89575),	  (-29.84375, 15.8645),		 (-29.84375, 21.15625),
        //				   (-25.875, 23.802),	  (-20.58325, 25.125),		 (-25.875, 29.09375),
        //				   (-20.58325, 31.7395),  (-11.0089998, 23.2290001), (-8.67700005, 21.15625),
        //				   (-6.03125, 21.15625),  (-7.35424995, 29.09375),	 (-3.38549995, 29.09375),
        //				   (1.90625, 30.41675),	  (5.875, 17.1875),			 (11.16675, 25.125),
        //				   (9.84375, 29.09375),	  (13.8125, 31.7395),		 (21.75, 30.41675),
        //				   (28.3644981, 26.448),  (25.71875, 18.5105),		 (24.3957481, 13.21875),
        //				   (17.78125, 11.89575),  (15.1355, 7.92700005),	 (5.875, 9.25),
        //				   (1.90625, 11.89575),	  (-3.25, 11.89575),		 (-3.25, 9.9375),
        //				   (-4.70825005, 9.25),	  (-8.67700005, 9.25),		 (-11.323, 11.89575),
        //				   (-13.96875, 11.89575), (-15.29175, 14.54175),	 (-19.2605, 14.54175)};

        Vec2[] points =
        [
            (-56.885498f, 12.8985004f), (-56.885498f, 16.2057495f), (56.885498f, 16.2057495f), (56.885498f, -16.2057514f),
            (51.5935059f, -16.2057514f), (43.6559982f, -10.9139996f), (35.7184982f, -10.9139996f), (27.7809982f, -10.9139996f),
            (21.1664963f, -14.2212505f), (11.9059982f, -16.2057514f), (0f, -16.2057514f), (-10.5835037f, -14.8827496f),
            (-17.1980019f, -13.5597477f), (-21.1665001f, -12.2370014f), (-25.1355019f, -9.5909977f), (-31.75f, -3.63799858f),
            (-38.3644981f, 6.2840004f), (-42.3334999f, 9.59125137f), (-47.625f, 11.5755005f), (-56.885498f, 12.8985004f),
        ];

        int count = points.Length;

        ChainDef chainDef = ChainDef.DefaultChainDef();
        chainDef.Points = points;
        chainDef.Count = count;
        chainDef.IsLoop = true;
        chainDef.Friction = 0.2f;

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        GroundId = Body.CreateBody(WorldId, bodyDef);

        ChainId = Shape.CreateChain(GroundId, chainDef);
    }

    protected void Launch()
    {
        if (BodyId.IsNotNull)
        {
            Body.DestroyBody(BodyId);
        }

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;
        bodyDef.Position = (-55.0f, 13.5f);
        BodyId = Body.CreateBody(WorldId, bodyDef);

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 1.0f;
        shapeDef.Friction = Friction;
        shapeDef.Restitution = Restitution;

        if (ShapeType == CircleShape)
        {
            Circle circle = ((0.0f, 0.0f), 0.5f);
            ShapeId = Shape.CreateCircleShape(BodyId, shapeDef, circle);
        }
        else if (ShapeType == CapsuleShape)
        {
            Capsule capsule = ((-0.5f, 0.0f), (0.5f, 0.0f), 0.25f);
            ShapeId = Shape.CreateCapsuleShape(BodyId, shapeDef, capsule);
        }
        else
        {
            float h = 0.5f;
            Polygon box = Geometry.MakeBox(h, h);
            ShapeId = Shape.CreatePolygonShape(BodyId, shapeDef, box);
        }
    }

    protected override void OnRender()
    {
        base.OnRender();

        Draw.DrawSegment(Vec2.Zero, (0.5f, 0.0f), B2HexColor.Red);
        Draw.DrawSegment(Vec2.Zero, (0.0f, 0.5f), B2HexColor.Green);
    }

    protected BodyId GroundId;

    protected BodyId BodyId;

    protected ChainId ChainId;

    protected int ShapeType;

    protected ShapeId ShapeId;

    protected float Restitution;

    protected float Friction;
}