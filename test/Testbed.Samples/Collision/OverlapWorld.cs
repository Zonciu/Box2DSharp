using System.Diagnostics;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Collision;

[Sample("Collision", "Overlap World")]
public class OverlapWorld : SampleBase
{
    protected int BodyIndex;

    protected BodyId[] BodyIds = new BodyId[MaxCount];

    protected ShapeUserData[] UserDatas = new ShapeUserData[MaxCount].Fill();

    protected Polygon[] Polygons = new Polygon[4];

    protected Capsule Capsule;

    protected Circle Circle;

    protected Segment Segment;

    protected int IgnoreIndex;

    protected ShapeId[] DoomIds = new ShapeId[MaxDoomed];

    protected int DoomCount;

    protected Circle QueryCircle;

    protected Capsule QueryCapsule;

    protected Polygon QueryBox;

    protected int ShapeType;

    protected Transform Transform;

    protected Vec2 StartPosition;

    protected Vec2 Position;

    protected Vec2 BasePosition;

    protected float Angle;

    protected float BaseAngle;

    protected bool Dragging;

    protected bool Rotating;

    protected static class TestShapeType
    {
        public const int CircleShape = 0;

        public const int CapsuleShape = 1;

        public const int BoxShape = 2;
    }

    // This shows how to filter a specific shape using using data.
    protected class ShapeUserData
    {
        public int Index;

        public bool Ignore;
    }

    protected const int MaxCount = 64;

    protected const int MaxDoomed = 16;

    static bool OverlapResultFcn(ShapeId shapeId, object context)
    {
        ShapeUserData userData = (ShapeUserData)Shape.GetUserData(shapeId);
        if (userData != null && userData.Ignore)
        {
            // continue the query
            return true;
        }

        OverlapWorld sample = (OverlapWorld)context;

        if (sample.DoomCount < MaxDoomed)
        {
            int index = sample.DoomCount;
            sample.DoomIds[index] = shapeId;
            sample.DoomCount += 1;
        }

        // continue the query
        return true;
    }

    public OverlapWorld(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 10.0f);
            Global.Camera.Zoom = 25.0f * 0.7f;
        }

        {
            Vec2[] vertices = [(-0.5f, 0.0f), (0.5f, 0.0f), (0.0f, 1.5f)];
            Hull hull = HullFunc.ComputeHull(vertices, 3);
            Polygons[0] = Geometry.MakePolygon(hull, 0.0f);
        }

        {
            Vec2[] vertices = [(-0.1f, 0.0f), (0.1f, 0.0f), (0.0f, 1.5f)];
            Hull hull = HullFunc.ComputeHull(vertices, 3);
            Polygons[1] = Geometry.MakePolygon(hull, 0.0f);
        }

        {
            float w = 1.0f;
            float b = w / (2.0f + MathF.Sqrt(2.0f));
            float s = MathF.Sqrt(2.0f) * b;

            Vec2[] vertices =
            [
                (0.5f * s, 0.0f), (0.5f * w, b), (0.5f * w, b + s), (0.5f * s, w),
                (-0.5f * s, w), (-0.5f * w, b + s), (-0.5f * w, b), (-0.5f * s, 0.0f)
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

        IgnoreIndex = 7;

        ShapeType = TestShapeType.CircleShape;

        QueryCircle = ((0.0f, 0.0f), 1.0f);
        QueryCapsule = ((-1.0f, 0.0f), (1.0f, 0.0f), 0.5f);
        QueryBox = Geometry.MakeBox(2.0f, 0.5f);

        Position = (0.0f, 10.0f);
        Angle = 0.0f;
        Dragging = false;
        Rotating = false;

        DoomCount = 0;

        CreateN(0, 10);
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
        UserDatas[BodyIndex].Index = BodyIndex;
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
                Dragging = true;
                Position = p;
            }
            else if (e.Modifiers.IsSet(KeyModifiers.Shift) && Dragging == false)
            {
                Rotating = true;
                StartPosition = p;
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
            Position = p;
        }
        else if (Rotating)
        {
            float dx = p.X - StartPosition.X;
            Angle = BaseAngle + 1.0f * dx;
        }
    }

    public override void PostStep()
    {
        DrawString("left mouse button: drag query shape");
        DrawString("left mouse button + shift: rotate query shape");

        DoomCount = 0;

        Transform transform = (Position, B2Math.MakeRot(Angle));

        if (ShapeType == TestShapeType.CircleShape)
        {
            World.OverlapCircle(
                WorldId,
                QueryCircle,
                transform,
                QueryFilter.DefaultQueryFilter(),
                OverlapResultFcn,
                this);
            Draw.DrawCircle(transform.P, QueryCircle.Radius, B2HexColor.White);
        }
        else if (ShapeType == TestShapeType.CapsuleShape)
        {
            World.OverlapCapsule(
                WorldId,
                QueryCapsule,
                transform,
                QueryFilter.DefaultQueryFilter(),
                OverlapResultFcn,
                this);
            Vec2 p1 = B2Math.TransformPoint(transform, QueryCapsule.Center1);
            Vec2 p2 = B2Math.TransformPoint(transform, QueryCapsule.Center2);
            Draw.DrawCapsule(p1, p2, QueryCapsule.Radius, B2HexColor.White);
        }
        else if (ShapeType == TestShapeType.BoxShape)
        {
            World.OverlapPolygon(
                WorldId,
                QueryBox,
                transform,
                QueryFilter.DefaultQueryFilter(),
                OverlapResultFcn,
                this);
            Vec2[] points = new Vec2[Core.MaxPolygonVertices];
            for (int i = 0; i < QueryBox.Count; ++i)
            {
                points[i] = B2Math.TransformPoint(transform, QueryBox.Vertices[i]);
            }

            Draw.DrawPolygon(points, QueryBox.Count, B2HexColor.White);
        }

        if (BodyIds[IgnoreIndex].IsNotNull)
        {
            Vec2 p = Body.GetPosition(BodyIds[IgnoreIndex]);
            p.X -= 0.2f;
            Draw.DrawString(p, "skip");
        }

        for (int i = 0; i < DoomCount; ++i)
        {
            ShapeId shapeId = DoomIds[i];
            ShapeUserData userData = (ShapeUserData)Shape.GetUserData(shapeId);
            if (userData == null)
            {
                continue;
            }

            int index = userData.Index;
            Debug.Assert(0 <= index && index < MaxCount);
            Debug.Assert(BodyIds[index].IsNotNull);

            Body.DestroyBody(BodyIds[index]);
            BodyIds[index] = BodyId.NullId;
        }
    }
}