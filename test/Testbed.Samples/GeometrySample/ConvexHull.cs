using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.GeometrySample;

[Sample("Geometry", "Convex Hull")]
public class ConvexHull : SampleBase
{
    protected Vec2[] Points = new Vec2[Core.MaxPolygonVertices];

    protected int Count;

    protected int Generation;

    protected bool Auto;

    protected bool Bulk;

    protected const int PointCount = Core.MaxPolygonVertices;

    public ConvexHull(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.5f, 0.0f);
            Global.Camera.Zoom = 25.0f * 0.3f;
        }

        Generation = 0;
        Auto = false;
        Bulk = false;
        Generate();
    }

    void Generate()
    {
        float angle = B2Math.Pi * RandomFloat();
        Rot r = B2Math.MakeRot(angle);

        Vec2 lowerBound = (-4.0f, -4.0f);
        Vec2 upperBound = (4.0f, 4.0f);

        for (int i = 0; i < PointCount; ++i)
        {
            float x = 10.0f * RandomFloat();
            float y = 10.0f * RandomFloat();

            // Clamp onto a square to help create collinearities.
            // This will stress the convex hull algorithm.
            Vec2 v = B2Math.Clamp((x, y), lowerBound, upperBound);
            Points[i] = B2Math.RotateVector(r, v);
        }

        Count = PointCount;

        Generation += 1;
    }

    public override void OnKeyDown(KeyInputEventArgs keyInput)
    {
        base.OnKeyDown(keyInput);
        switch (keyInput.Key)
        {
        case KeyCodes.A:
            Auto = !Auto;
            break;
        case KeyCodes.B:
            Bulk = !Bulk;
            break;
        case KeyCodes.G:
            Generate();
            break;
        default:
            break;
        }
    }

    protected override void OnRender()
    {
        DrawString("Options: generate(g), auto(a), bulk(b)");

        Hull hull = new();
        bool valid = false;
        float milliseconds = 0.0f;

        if (Bulk)
        {
            // defect hunting
            for (int i = 0; i < 10000; ++i)
            {
                Generate();
                hull = HullFunc.ComputeHull(Points, Count);
                if (hull.Count == 0)
                {
                    // m_bulk = false;
                    // break;
                    continue;
                }

                valid = HullFunc.ValidateHull(hull);
                if (valid == false || Bulk == false)
                {
                    Bulk = false;
                    break;
                }
            }
        }
        else
        {
            if (Auto)
            {
                Generate();
            }

            hull = HullFunc.ComputeHull(Points, Count);
            if (hull.Count > 0)
            {
                valid = HullFunc.ValidateHull(hull);
                if (valid == false)
                {
                    Auto = false;
                }
            }
        }

        if (valid == false)
        {
            DrawString($"generation = {Generation}, FAILED");
        }
        else
        {
            DrawString($"generation = {Generation}, count = {hull.Count}");
        }

        if (milliseconds > 0.0f)
        {
            DrawString($"milliseconds = {milliseconds}");
        }

        Draw.DrawPolygon(hull.Points, hull.Count, B2HexColor.Gray);

        for (int i = 0; i < Count; ++i)
        {
            Draw.DrawPoint(Points[i], 5.0f, B2HexColor.Blue);
            Draw.DrawString(Points[i] + (0.1f, 0.1f), i.ToString());
        }

        for (int i = 0; i < hull.Count; ++i)
        {
            Draw.DrawPoint(hull.Points[i], 6.0f, B2HexColor.Green);
        }
    }
}