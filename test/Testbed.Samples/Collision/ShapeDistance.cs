using System.Diagnostics;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Collision;

[Sample("Collision", "Shape Distance")]
public class ShapeDistance : SampleBase
{
    const int SIMPLEX_CAPACITY = 20;

    protected Polygon Box;

    protected Polygon Triangle;

    protected Vec2 Point;

    protected Segment Segment;

    protected TestShapeType TypeA;

    protected TestShapeType TypeB;

    protected float RadiusA;

    protected float RadiusB;

    protected DistanceProxy ProxyA;

    protected DistanceProxy ProxyB;

    protected DistanceCache Cache;

    protected Simplex[] Simplexes = new Simplex[SIMPLEX_CAPACITY];

    protected int SimplexCount;

    protected int SimplexIndex;

    protected Transform Transform;

    protected float Angle;

    protected Vec2 BasePosition;

    protected Vec2 StartPoint;

    protected float BaseAngle;

    protected bool Dragging;

    protected bool Rotating;

    protected bool ShowIndices;

    protected bool UseCache;

    protected bool DrawSimplex;

    protected enum TestShapeType
    {
        Point,

        Segment,

        Triangle,

        Box
    };

    public ShapeDistance(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 0.0f);
            Global.Camera.Zoom = 3.0f;
        }

        Point = Vec2.Zero;
        Segment = ((-0.5f, 0.0f), (0.5f, 0.0f));

        {
            Vec2[] points = [(-0.5f, 0.0f), (0.5f, 0.0f), (0.0f, 1.0f)];
            Hull hull = HullFunc.ComputeHull(points, 3);
            Triangle = Geometry.MakePolygon(hull, 0.0f);
        }

        Box = Geometry.MakeBox(0.5f, 0.5f);

        Transform = ((1.5f, -1.5f), Rot.Identity);
        Angle = 0.0f;

        Cache = new DistanceCache();
        SimplexCount = 0;
        StartPoint = (0.0f, 0.0f);
        BasePosition = (0.0f, 0.0f);
        BaseAngle = 0.0f;

        Dragging = false;
        Rotating = false;
        ShowIndices = false;
        UseCache = false;
        DrawSimplex = false;

        TypeA = TestShapeType.Box;
        TypeB = TestShapeType.Box;
        RadiusA = 0.0f;
        RadiusB = 0.0f;

        ProxyA = MakeProxy(TypeA, RadiusA);
        ProxyB = MakeProxy(TypeB, RadiusB);
    }

    protected DistanceProxy MakeProxy(TestShapeType type, float radius)
    {
        DistanceProxy proxy = new();
        proxy.Radius = radius;

        switch (type)
        {
        case TestShapeType.Point:
            proxy.Points[0] = Vec2.Zero;
            proxy.Count = 1;
            break;

        case TestShapeType.Segment:
            proxy.Points[0] = Segment.Point1;
            proxy.Points[1] = Segment.Point2;
            proxy.Count = 2;
            break;

        case TestShapeType.Triangle:
            proxy.Points[0] = Triangle.Vertices[0];
            proxy.Points[1] = Triangle.Vertices[1];
            proxy.Points[2] = Triangle.Vertices[2];
            proxy.Count = 3;
            break;

        case TestShapeType.Box:
            proxy.Points[0] = Box.Vertices[0];
            proxy.Points[1] = Box.Vertices[1];
            proxy.Points[2] = Box.Vertices[2];
            proxy.Points[3] = Box.Vertices[3];
            proxy.Count = 4;
            break;

        default:
            Debug.Assert(false);
            break;
        }

        return proxy;
    }

    void DrawShape(TestShapeType type, Transform transform, float radius, B2HexColor color)
    {
        switch (type)
        {
        case TestShapeType.Point:
        {
            Vec2 p = B2Math.TransformPoint(transform, Point);
            if (radius > 0.0f)
            {
                Draw.DrawCircle(p, radius, color);
            }
            else
            {
                Draw.DrawPoint(p, 5.0f, color);
            }
        }
            break;

        case TestShapeType.Segment:
        {
            Vec2 p1 = B2Math.TransformPoint(transform, Segment.Point1);
            Vec2 p2 = B2Math.TransformPoint(transform, Segment.Point2);

            if (radius > 0.0f)
            {
                Draw.DrawCapsule(p1, p2, radius, color);
            }
            else
            {
                Draw.DrawSegment(p1, p2, color);
            }
        }
            break;

        case TestShapeType.Triangle:
            Draw.DrawSolidPolygon(transform, Triangle.Vertices, 3, radius, color);
            break;

        case TestShapeType.Box:
            Draw.DrawSolidPolygon(transform, Box.Vertices, 4, radius, color);
            break;

        default:
            Debug.Assert(false);
            break;
        }
    }

    public override void MouseDown(Vec2 p, MouseInputEventArgs e)
    {
        if (e.Button == MouseButton.Left)
        {
            if (!e.Modifiers.IsSet(KeyModifiers.Shift) && Rotating == false)
            {
                Dragging = true;
                StartPoint = p;
                BasePosition = Transform.P;
            }
            else if (e.Modifiers.IsSet(KeyModifiers.Shift) && Dragging == false)
            {
                Rotating = true;
                StartPoint = p;
                BaseAngle = Angle;
            }
        }
    }

    public override void MouseUp(Vec2 p, MouseInputEventArgs e)
    {
        if (e.Button == MouseButton.Left)
        {
            Dragging = false;
            Rotating = false;
        }
    }

    public override void MouseMove(Vec2 p, MouseMoveEventArgs e)
    {
        if (Dragging)
        {
            Transform.P.X = BasePosition.X + 0.5f * (p.X - StartPoint.X);
            Transform.P.Y = BasePosition.Y + 0.5f * (p.Y - StartPoint.Y);
        }
        else if (Rotating)
        {
            float dx = p.X - StartPoint.X;
            Angle = Math.Clamp(BaseAngle + 1.0f * dx, -B2Math.Pi, B2Math.Pi);
            Transform.Q = B2Math.MakeRot(Angle);
        }
    }

    static Vec2 Weight2(float a1, Vec2 w1, float a2, Vec2 w2)
    {
        return (a1 * w1.X + a2 * w2.X, a1 * w1.Y + a2 * w2.Y);
    }

    static Vec2 Weight3(float a1, Vec2 w1, float a2, Vec2 w2, float a3, Vec2 w3)
    {
        return (a1 * w1.X + a2 * w2.X + a3 * w3.X, a1 * w1.Y + a2 * w2.Y + a3 * w3.Y);
    }

    void ComputeSimplexWitnessPoints(out Vec2 a, out Vec2 b, in Simplex s)
    {
        switch (s.Count)
        {
        case 0:
            Debug.Assert(false);
            a = default;
            b = default;
            break;

        case 1:
            a = s.V1.WA;
            b = s.V1.WB;
            break;

        case 2:
            a = Weight2(s.V1.A, s.V1.WA, s.V2.A, s.V2.WA);
            b = Weight2(s.V1.A, s.V1.WB, s.V2.A, s.V2.WB);
            break;

        case 3:
            a = Weight3(s.V1.A, s.V1.WA, s.V2.A, s.V2.WA, s.V3.A, s.V3.WA);
            b = a;
            break;

        default:
            Debug.Assert(false);
            a = default;
            b = default;
            break;
        }
    }

    public override void PreStep()
    {
        DistanceInput input;
        input.ProxyA = ProxyA;
        input.ProxyB = ProxyB;
        input.TransformA = Transform.Identity;
        input.TransformB = Transform;
        input.UseRadii = RadiusA > 0.0f || RadiusB > 0.0f;

        if (UseCache == false)
        {
            Cache.Count = 0;
        }

        DistanceOutput output = DistanceFunc.ShapeDistance(ref Cache, input, Simplexes, SIMPLEX_CAPACITY);

        SimplexCount = output.SimplexCount;

        DrawShape(TypeA, Transform.Identity, RadiusA, B2HexColor.Cyan);
        DrawShape(TypeB, Transform, RadiusB, B2HexColor.Bisque);

        if (DrawSimplex)
        {
            ref readonly Simplex simplex = ref Simplexes[SimplexIndex];
            var vertices = simplex.Vertices;

            if (SimplexIndex > 0)
            {
                // The first recorded simplex does not have valid barycentric coordinates
                ComputeSimplexWitnessPoints(out var pointA, out var pointB, simplex);

                Draw.DrawSegment(pointA, pointB, B2HexColor.White);
                Draw.DrawPoint(pointA, 5.0f, B2HexColor.White);
                Draw.DrawPoint(pointB, 5.0f, B2HexColor.White);
            }

            B2HexColor[] colors = [B2HexColor.Red, B2HexColor.Green, B2HexColor.Blue];

            for (int i = 0; i < simplex.Count; ++i)
            {
                ref readonly SimplexVertex vertex = ref vertices[i];
                Draw.DrawPoint(vertex.WA, 5.0f, colors[i]);
                Draw.DrawPoint(vertex.WB, 5.0f, colors[i]);
            }
        }
        else
        {
            Draw.DrawSegment(output.PointA, output.PointB, B2HexColor.White);
            Draw.DrawPoint(output.PointA, 5.0f, B2HexColor.White);
            Draw.DrawPoint(output.PointB, 5.0f, B2HexColor.White);
        }

        if (ShowIndices)
        {
            for (int i = 0; i < ProxyA.Count; ++i)
            {
                Vec2 p = ProxyA.Points[i];
                Draw.DrawString(p, $" {i}");
            }

            for (int i = 0; i < ProxyB.Count; ++i)
            {
                Vec2 p = B2Math.TransformPoint(Transform, ProxyB.Points[i]);
                Draw.DrawString(p, $" {i}");
            }
        }

        DrawString("mouse button 1: drag");
        DrawString("mouse button 1 + shift: rotate");
        DrawString($"distance = {output.Distance:F2}, iterations = {output.Iterations}");

        if (Cache.Count == 1)
        {
            DrawString($"cache = {{{Cache.IndexA[0]}, {Cache.IndexB[0]}}}");
        }
        else if (Cache.Count == 2)
        {
            DrawString($"cache = {{{Cache.IndexA[0]}, {Cache.IndexA[1]}}}, {{{Cache.IndexB[0]}, {Cache.IndexB[1]}}}");
        }
        else if (Cache.Count == 3)
        {
            DrawString($"cache = {{{Cache.IndexA[0]}, {Cache.IndexA[1]}, {Cache.IndexA[2]}}}, {{{Cache.IndexB[0]}, {Cache.IndexB[1]}, {Cache.IndexB[2]}}}");
        }
    }
}