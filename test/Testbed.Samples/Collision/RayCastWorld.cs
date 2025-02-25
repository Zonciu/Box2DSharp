using System.Diagnostics;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Collision;

[Sample("Collision", "Ray Cast World")]
public class RayCastWorld : SampleBase
{
    protected int BodyIndex;

    protected BodyId[] BodyIds = new BodyId[MaxCount];

    protected ShapeUserData[] UserDatas = new ShapeUserData[MaxCount].Fill();

    protected Polygon[] Polygons = new Polygon[4];

    protected Capsule Capsule;

    protected Circle Circle;

    protected Segment Segment;

    protected bool Simple;

    protected TestCastMode Mode;

    protected int IgnoreIndex;

    protected TestCastType CastType;

    protected float CastRadius;

    protected Vec2 AngleAnchor;

    protected float BaseAngle;

    protected float Angle;

    protected bool Rotating;

    protected Vec2 RayStart;

    protected Vec2 RayEnd;

    protected bool Dragging;

    // This shows how to filter a specific shape using using data.
    protected class ShapeUserData
    {
        public int Index;

        public bool Ignore;
    }

    // Context for ray cast callbacks. Do what you want with this.
    protected class RayCastContext
    {
        public Vec2[] Points = new Vec2[3];

        public Vec2[] Normals = new Vec2[3];

        public float[] Fractions = new float[3];

        public int Count;
    };

    protected enum TestCastMode
    {
        Any = 0,

        Closest = 1,

        Multiple = 2,

        Sorted = 3
    }

    protected enum TestCastType
    {
        RayCast = 0,

        CircleCast = 1,

        CapsuleCast = 2,

        PolygonCast = 3
    }

    public const int MaxCount = 64;

    public RayCastWorld(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (2.0f, 14.0f);
            Global.Camera.Zoom = 25.0f * 0.75f;
        }

        // Ground body
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Segment segment = ((-40.0f, 0.0f), (40.0f, 0.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }

        {
            Vec2[] vertices = [(-0.5f, 0.0f), (0.5f, 0.0f), (0.0f, 1.5f)];
            Hull hull = HullFunc.ComputeHull(vertices, 3);
            Polygons[0] = Geometry.MakePolygon(hull, 0.0f);
        }

        {
            Vec2[] vertices =
                [
                    (
                        -0.1f, 0.0f
                    ),
                    (
                        0.1f, 0.0f
                    ),
                    (
                        0.0f, 1.5f
                    )
                ]
                ;
            Hull hull = HullFunc.ComputeHull(vertices, 3);
            Polygons[1] = Geometry.MakePolygon(hull, 0.0f);
            Polygons[1].Radius = 0.5f;
        }

        {
            float w = 1.0f;
            float b = w / (2.0f + MathF.Sqrt(2.0f));
            float s = MathF.Sqrt(2.0f) * b;

            Vec2[] vertices =
            [
                (0.5f * s, 0.0f),
                (0.5f * w, b),
                (0.5f * w, b + s),
                (0.5f * s, w),
                (-0.5f * s, w),
                (-0.5f * w, b + s),
                (-0.5f * w, b),
                (-0.5f * s, 0.0f)
            ];

            Hull hull = HullFunc.ComputeHull(vertices, 8);
            Polygons[2] = Geometry.MakePolygon(hull, 0.0f);
        }

        Polygons[3] = Geometry.MakeBox(0.5f, 0.5f);
        Capsule = ((-0.5f, 0.0f), (0.5f, 0.0f), 0.25f);
        Circle = ((0.0f, 0.0f), 0.5f);
        Segment = ((-1.0f, 0.0f), (1.0f, 0.0f));
        BodyIndex = 0;

        for (int i = 0; i < MaxCount; ++i)
        {
            BodyIds[i] = BodyId.NullId;
        }

        Mode = TestCastMode.Closest;
        IgnoreIndex = 7;

        CastType = TestCastType.RayCast;
        CastRadius = 0.5f;

        RayStart = (-20.0f, 10.0f);
        RayEnd = (20.0f, 10.0f);
        Dragging = false;

        Angle = 0.0f;
        BaseAngle = 0.0f;
        AngleAnchor = (0.0f, 0.0f);
        Rotating = false;

        Simple = false;
    }

    protected void Create(int index)
    {
        if (BodyIds[BodyIndex].IsNotNull)
        {
            Body.DestroyBody(BodyIds[BodyIndex]);
            BodyIds[BodyIndex] = BodyId.NullId;
        }

        float x = RandomFloat(-20.0f, 20.0f);
        float y = RandomFloat(0.0f, 20.0f);

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Position = (x, y);
        bodyDef.Rotation = B2Math.MakeRot(RandomFloat(-B2Math.Pi, B2Math.Pi));

        BodyIds[BodyIndex] = Body.CreateBody(WorldId, bodyDef);

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.UserData = UserDatas[BodyIndex];
        UserDatas[BodyIndex].Ignore = false;
        if (BodyIndex == IgnoreIndex)
        {
            UserDatas[BodyIndex].Ignore = true;
        }

        if (index < 4)
        {
            Shape.CreatePolygonShape(BodyIds[BodyIndex], shapeDef, Polygons[index]);
        }
        else if (index == 4)
        {
            Shape.CreateCircleShape(BodyIds[BodyIndex], shapeDef, Circle);
        }
        else if (index == 5)
        {
            Shape.CreateCapsuleShape(BodyIds[BodyIndex], shapeDef, Capsule);
        }
        else
        {
            Shape.CreateSegmentShape(BodyIds[BodyIndex], shapeDef, Segment);
        }

        BodyIndex = (BodyIndex + 1) % MaxCount;
    }

    protected void CreateN(int index, int count)
    {
        for (int i = 0; i < count; ++i)
        {
            Create(index);
        }
    }

    protected void DestroyBody()
    {
        for (int i = 0; i < MaxCount; ++i)
        {
            if (BodyIds[i].IsNotNull)
            {
                Body.DestroyBody(BodyIds[i]);
                BodyIds[i] = BodyId.NullId;
                return;
            }
        }
    }

    public override void MouseDown(Vec2 p, MouseInputEventArgs e)
    {
        base.MouseDown(p, e);

        if (e.Button == MouseButton.Left)
        {
            if (!e.Modifiers.IsSet(KeyModifiers.Shift) && Rotating == false)
            {
                RayStart = p;
                RayEnd = p;
                Dragging = true;
            }
            else if (e.Modifiers.IsSet(KeyModifiers.Shift) && Dragging == false)
            {
                Rotating = true;
                AngleAnchor = p;
                BaseAngle = Angle;
            }
        }
    }

    public override void MouseUp(Vec2 p, MouseInputEventArgs e)
    {
        base.MouseUp(p, e);

        if (e.Button == MouseButton.Left)
        {
            Dragging = false;
            Rotating = false;
        }
    }

    public override void MouseMove(Vec2 p, MouseMoveEventArgs e)
    {
        base.MouseMove(p, e);

        if (Dragging)
        {
            RayEnd = p;
        }
        else if (Rotating)
        {
            float dx = p.X - AngleAnchor.X;
            Angle = BaseAngle + 1.0f * dx;
        }
    }

    public override void PostStep()
    {
        DrawString("Click left mouse button and drag to modify ray cast");
        DrawString("Shape 7 is intentionally ignored by the ray");

        const B2HexColor color1 = B2HexColor.Green;
        const B2HexColor color2 = B2HexColor.Gray8;
        const B2HexColor color3 = B2HexColor.Magenta;

        Vec2 rayTranslation = RayEnd - RayStart;

        if (Simple)
        {
            DrawString("Simple closest point ray cast");

            // This version doesn't have a callback, but it doesn't skip the ignored shape
            using RayResult result = World.CastRayClosest(WorldId, RayStart, rayTranslation, QueryFilter.DefaultQueryFilter());

            if (result.Hit == true)
            {
                Vec2 c = B2Math.MulAdd(RayStart, result.Fraction, rayTranslation);
                Draw.DrawPoint(result.Point, 5.0f, color1);
                Draw.DrawSegment(RayStart, c, color2);
                Vec2 head = B2Math.MulAdd(result.Point, 0.5f, result.Normal);
                Draw.DrawSegment(result.Point, head, color3);
            }
            else
            {
                Draw.DrawSegment(RayStart, RayEnd, color2);
            }
        }
        else
        {
            switch (Mode)
            {
            case TestCastMode.Any:
                DrawString("Cast mode: any - check for obstruction - unsorted");
                break;

            case TestCastMode.Closest:
                DrawString("Cast mode: closest - find closest shape along the cast");
                break;

            case TestCastMode.Multiple:
                DrawString("Cast mode: multiple - gather up to 3 shapes - unsorted");
                break;

            case TestCastMode.Sorted:
                DrawString("Cast mode: sorted - gather up to 3 shapes sorted by closeness");
                break;
            }

            CastResultFcn modeFcn = Mode switch
            {
                TestCastMode.Any => _rayCastAnyCallback,
                TestCastMode.Closest => _rayCastClosestCallback,
                TestCastMode.Multiple => _rayCastMultipleCallback,
                TestCastMode.Sorted => _rayCastSortedCallback,
                _ => throw new ArgumentOutOfRangeException()
            };

            RayCastContext context = new();

            // Must initialize fractions for sorting
            context.Fractions[0] = float.MaxValue;
            context.Fractions[1] = float.MaxValue;
            context.Fractions[2] = float.MaxValue;

            Circle circle = ((0.0f, 0.0f), CastRadius);
            Capsule capsule = ((-0.25f, 0.0f), (0.25f, 0.0f), CastRadius);
            Polygon box = Geometry.MakeRoundedBox(0.25f, 0.5f, CastRadius);
            Transform transform = (RayStart, B2Math.MakeRot(Angle));

            switch (CastType)
            {
            case TestCastType.RayCast:
                World.CastRay(WorldId, RayStart, rayTranslation, QueryFilter.DefaultQueryFilter(), modeFcn, context);
                break;

            case TestCastType.CircleCast:
                World.CastCircle(
                    WorldId,
                    circle,
                    transform,
                    rayTranslation,
                    QueryFilter.DefaultQueryFilter(),
                    modeFcn,
                    context);
                break;

            case TestCastType.CapsuleCast:
                World.CastCapsule(
                    WorldId,
                    capsule,
                    transform,
                    rayTranslation,
                    QueryFilter.DefaultQueryFilter(),
                    modeFcn,
                    context);
                break;

            case TestCastType.PolygonCast:
                World.CastPolygon(WorldId, box, transform, rayTranslation, QueryFilter.DefaultQueryFilter(), modeFcn, context);
                break;
            }

            if (context.Count > 0)
            {
                Debug.Assert(context.Count <= 3);
                B2HexColor[] colors = [B2HexColor.Red, B2HexColor.Green, B2HexColor.Blue];
                for (int i = 0; i < context.Count; ++i)
                {
                    Vec2 c = B2Math.MulAdd(RayStart, context.Fractions[i], rayTranslation);
                    Vec2 p = context.Points[i];
                    Vec2 n = context.Normals[i];
                    Draw.DrawPoint(p, 5.0f, colors[i]);
                    Draw.DrawSegment(RayStart, c, color2);
                    Vec2 head = B2Math.MulAdd(p, 0.5f, n);
                    Draw.DrawSegment(p, head, color3);

                    Vec2 t = B2Math.MulSV(context.Fractions[i], rayTranslation);

                    if (CastType == TestCastType.CircleCast)
                    {
                        Draw.DrawCircle(RayStart + t, CastRadius, B2HexColor.Yellow);
                    }
                    else if (CastType == TestCastType.CapsuleCast)
                    {
                        Vec2 p1 = B2Math.TransformPoint(transform, capsule.Center1) + t;
                        Vec2 p2 = B2Math.TransformPoint(transform, capsule.Center2) + t;
                        Draw.DrawCapsule(p1, p2, CastRadius, B2HexColor.Yellow);
                    }
                    else if (CastType == TestCastType.PolygonCast)
                    {
                        Transform shiftedTransform = ((transform.P + t), transform.Q);
                        Draw.DrawSolidPolygon(shiftedTransform, box.Vertices, box.Count, box.Radius, B2HexColor.Yellow);
                    }
                }
            }
            else
            {
                Draw.DrawSegment(RayStart, RayEnd, color2);

                if (CastType == TestCastType.CircleCast)
                {
                    Draw.DrawCircle((RayStart + rayTranslation), CastRadius, B2HexColor.Gray);
                }
                else if (CastType == TestCastType.CapsuleCast)
                {
                    Vec2 p1 = B2Math.TransformPoint(transform, capsule.Center1) + rayTranslation;
                    Vec2 p2 = B2Math.TransformPoint(transform, capsule.Center2) + rayTranslation;
                    Draw.DrawCapsule(p1, p2, CastRadius, B2HexColor.Yellow);
                }
                else if (CastType == TestCastType.PolygonCast)
                {
                    Transform shiftedTransform = ((transform.P + rayTranslation), transform.Q);
                    Draw.DrawSolidPolygon(shiftedTransform, box.Vertices, box.Count, box.Radius, B2HexColor.Yellow);
                }
            }
        }

        Draw.DrawPoint(RayStart, 5.0f, B2HexColor.Green);

        if (BodyIds[IgnoreIndex].IsNotNull)
        {
            Vec2 p = Body.GetPosition(BodyIds[IgnoreIndex]);
            p.X -= 0.2f;
            Draw.DrawString(p, "ign");
        }
    }

    // This callback finds the closest hit. This is the most common callback used in games.
    private static readonly CastResultFcn _rayCastClosestCallback = (shapeId, point, normal, fraction, context) =>
    {
        RayCastContext rayContext = (RayCastContext)context;

        ShapeUserData? userData = (ShapeUserData?)Shape.GetUserData(shapeId);
        if (userData is { Ignore: true })
        {
            // By returning -1, we instruct the calling code to ignore this shape and
            // continue the ray-cast to the next shape.
            return -1.0f;
        }

        rayContext.Points[0] = point;
        rayContext.Normals[0] = normal;
        rayContext.Fractions[0] = fraction;
        rayContext.Count = 1;

        // By returning the current fraction, we instruct the calling code to clip the ray and
        // continue the ray-cast to the next shape. WARNING: do not assume that shapes
        // are reported in order. However, by clipping, we can always get the closest shape.
        return fraction;
    };

    // This callback finds any hit. For this type of query we are usually just checking for obstruction,
    // so the hit data is not relevant.
    // NOTE: shape hits are not ordered, so this may not return the closest hit
    private static readonly CastResultFcn _rayCastAnyCallback = (shapeId, point, normal, fraction, context) =>
    {
        RayCastContext rayContext = (RayCastContext)context;

        ShapeUserData? userData = (ShapeUserData?)Shape.GetUserData(shapeId);
        if (userData is { Ignore: true })
        {
            // By returning -1, we instruct the calling code to ignore this shape and
            // continue the ray-cast to the next shape.
            return -1.0f;
        }

        rayContext.Points[0] = point;
        rayContext.Normals[0] = normal;
        rayContext.Fractions[0] = fraction;
        rayContext.Count = 1;

        // At this point we have a hit, so we know the ray is obstructed.
        // By returning 0, we instruct the calling code to terminate the ray-cast.
        return 0.0f;
    };

    // This ray cast collects multiple hits along the ray.
    // The shapes are not necessary reported in order, so we might not capture
    // the closest shape.
    // NOTE: shape hits are not ordered, so this may return hits in any order. This means that
    // if you limit the number of results, you may discard the closest hit. You can see this
    // behavior in the sample.
    private static readonly CastResultFcn _rayCastMultipleCallback = (shapeId, point, normal, fraction, context) =>
    {
        RayCastContext rayContext = (RayCastContext)context;

        ShapeUserData? userData = (ShapeUserData?)Shape.GetUserData(shapeId);
        if (userData is { Ignore: true })
        {
            // By returning -1, we instruct the calling code to ignore this shape and
            // continue the ray-cast to the next shape.
            return -1.0f;
        }

        int count = rayContext.Count;
        Debug.Assert(count < 3);

        rayContext.Points[count] = point;
        rayContext.Normals[count] = normal;
        rayContext.Fractions[count] = fraction;
        rayContext.Count = count + 1;

        if (rayContext.Count == 3)
        {
            // At this point the buffer is full.
            // By returning 0, we instruct the calling code to terminate the ray-cast.
            return 0.0f;
        }

        // By returning 1, we instruct the caller to continue without clipping the ray.
        return 1.0f;
    };

    // This ray cast collects multiple hits along the ray and sorts them.
    private static readonly CastResultFcn _rayCastSortedCallback = (shapeId, point, normal, fraction, context) =>
    {
        RayCastContext rayContext = (RayCastContext)context;

        ShapeUserData? userData = (ShapeUserData?)Shape.GetUserData(shapeId);
        if (userData is { Ignore: true })
        {
            // By returning -1, we instruct the calling code to ignore this shape and
            // continue the ray-cast to the next shape.
            return -1.0f;
        }

        int count = rayContext.Count;
        Debug.Assert(count <= 3);

        int index = 3;
        while (fraction < rayContext.Fractions[index - 1])
        {
            index -= 1;

            if (index == 0)
            {
                break;
            }
        }

        if (index == 3)
        {
            // not closer, continue but tell the caller not to consider fractions further than the largest fraction acquired
            // this only happens once the buffer is full
            Debug.Assert(rayContext.Count == 3);
            Debug.Assert(rayContext.Fractions[2] <= 1.0f);
            return rayContext.Fractions[2];
        }

        for (int j = 2; j > index; --j)
        {
            rayContext.Points[j] = rayContext.Points[j - 1];
            rayContext.Normals[j] = rayContext.Normals[j - 1];
            rayContext.Fractions[j] = rayContext.Fractions[j - 1];
        }

        rayContext.Points[index] = point;
        rayContext.Normals[index] = normal;
        rayContext.Fractions[index] = fraction;
        rayContext.Count = count < 3 ? count + 1 : 3;

        if (rayContext.Count == 3)
        {
            return rayContext.Fractions[2];
        }

        // By returning 1, we instruct the caller to continue without clipping the ray.
        return 1.0f;
    };
}